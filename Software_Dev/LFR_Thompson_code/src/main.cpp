#include <Arduino.h>
#include <QTRSensors.h>
#include <BananaBar.h>
#include <TB6612.h>
#include <InterCom.h>
#include <CMSIS_DSP.h>
#include <Servo.h>

/*
  ---PINOUT ver. final v1 , tb6612, para barras de 8 y 16, con encoders---
*/
const uint8_t led_pin = PB13, led2_pin = PC13, start_IR_pin = PB12;
const uint8_t boton_1_pin = PB1, boton_2_pin = PC14 , turbina_pin = PB8;


uint16_t sensorValues[8];

TB6612_pinout motors_pinout = { 
  .PWMA_pin = PA8,
  .AIN2_pin = PA9,
  .AIN1_pin = PA10,
  .STBY_pin = PA15,
  .BIN1_pin = PB4,
  .BIN2_pin = PB3,
  .PWMB_pin = PB5
};

const uint8_t enc_izq_pins[2] ={PB14,PB15} , enc_der_pins [2] ={PB10,PB2};

TB6612 driver_motores(motors_pinout);
/*  //PARA BARRA DE 16 sensores;
Bar_Sensor_pinout barra_pinout{
  .mux_read_pin = PA2,
  .mux_selec_pins = {PA3,PA4,PA5,PA6},
  .Weighs_curve = {-80,-70,-60,-50,-40,-30,-20,-10,10,20,30,40,50,60,70,80}
};
Bar_Sensors barra(&barra_pinout); */

QTRSensors barra;
uint16_t vs[8] ={0};

SimpleCommand cmd;

HardwareTimer *pid_INT = new HardwareTimer(TIM10);
const int hz_control= 200;

arm_pid_instance_f32 pid_parametes;

bool check = false, check2 = false, check3 = false;
float setpoint=0;

Servo turbina;
bool en_tubina = false;
float inc_us = 100;

#define debug_port Serial1

void refresh(){
  arm_pid_init_f32(&pid_parametes, 1);
}

void values(){
  barra.read(sensorValues);
  
  for(int i = 0; i < 8; i++){
    debug_port.print(sensorValues[i]);
    debug_port.print("\t");
  }
  debug_port.print("\n");
}

void list(){
  cmd.list();
}

void config_turbina(){
  turbina.attach(turbina_pin);
  turbina.writeMicroseconds(1000);
}

void calibrar_tubina(){
  turbina.writeMicroseconds(1000);
  delay(1000);
  turbina.writeMicroseconds(2000);
  delay(1000);
  turbina.writeMicroseconds(1000);
}

void turbina_enable(){
  en_tubina = !en_tubina;
  if(en_tubina){
    for(long i = 0; i<1000;i++){
      turbina.writeMicroseconds(1000+i);
      //delay(2);
    }
    turbina.writeMicroseconds(2000);
  }
  else{
    turbina.writeMicroseconds(1000);
  }
}

void calibrarBarra(){
  digitalWrite(led_pin,HIGH);
  for(uint8_t i = 0; i<100; i++){
    barra.calibrate();
    delay(20);
  }
  digitalWrite(led_pin,LOW);
}

void config_barra(){
  barra.setTypeAnalog();
  barra.setSensorPins((const uint8_t[]){PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7}, 8);
  barra.setEmitterPin(PC15);
}

void readpos(){
  int pos = barra.readLineBlack(vs);
  debug_port.println(pos);
}

void doCheck(){
  check = !check;
}

void doCheck2(){
  check2 = !check2;
}

void doCheck3(){
  check3 = !check3;
}

void config_commands(){
  cmd.enable_echo(true);
  cmd.addCommand("list",list);
  cmd.addCommand("readpos",doCheck3);
  cmd.addCommand("check2",doCheck2);
  cmd.addCommand("check",doCheck);
  cmd.addCommand("enturbina",turbina_enable);
  cmd.addCommand("turcali", calibrar_tubina);
  cmd.addCommand("cali",calibrarBarra);
  cmd.addCommand("refresh", refresh);
  cmd.addCommand("t",&inc_us);
  cmd.addCommand("p",&pid_parametes.Kp);
  cmd.addCommand("i",&pid_parametes.Ki);
  cmd.addCommand("d",&pid_parametes.Kd);
  cmd.addCommand("s",&setpoint);
  cmd.begin(&debug_port);
}

void PID_ISR(){

  if(check){
    driver_motores.enableDriver(1);
    float error = (3500.0f-(float)barra.readLineBlack(vs))/100.0f;
    float inc_pwm = arm_pid_f32(&pid_parametes, error);
    if(inc_pwm < 0){
      driver_motores.setPWM((int)(setpoint + inc_pwm), (int)(setpoint));
    }
    else{
      driver_motores.setPWM((int)(setpoint), (int)(setpoint - inc_pwm));
    }
  }
  else{
    driver_motores.setPWM(0,0);
    //driver_motores.enableDriver(0);
  }

}

void config_timer_interrup(){
  
  pid_INT->pause();
  pid_INT->setMode(1,TIMER_DISABLED);   //Config para que solo sirva de contador para interrupt
  pid_INT->setOverflow((uint32_t)hz_control,HERTZ_FORMAT); //Interrupt 1 veces por segundo.
  pid_INT->attachInterrupt(1,PID_ISR);

  pid_INT->refresh();

}

void config_PID(){
  pid_parametes.Kp = 15.0f;
  pid_parametes.Ki = 0.0f;
  pid_parametes.Kd = 10.0f;
  arm_pid_init_f32(&pid_parametes,(int32_t)1);
}

void setup(){

  //---Inicializacion de pines---
  pinMode(turbina_pin,OUTPUT);
  pinMode(led_pin,OUTPUT);
  pinMode(led2_pin,OUTPUT);
  digitalWrite(led2_pin,HIGH);
  pinMode(boton_1_pin,INPUT);
  pinMode(boton_2_pin,INPUT);
  pinMode(start_IR_pin,INPUT);

  config_barra();
  driver_motores.begin();
  
  //---Inicializacion de protocolos de comunicacion---
  SerialUSB.begin(115200);  

  Serial1.setTx(PB6);
  Serial1.setRx(PB7);
  Serial1.begin(9600);

  //---Configuracion de la logica---
  config_commands();
  config_timer_interrup();
  config_turbina();
  config_PID();

  //while(digitalRead(boton_1_pin)) cmd.listen();
  calibrarBarra();
  while(!digitalRead(start_IR_pin)) cmd.listen();
  driver_motores.enableDriver(true);

  pid_INT->resume();  //Iniciando el TIMER para la interupcion del PID
}

void loop(){
  cmd.listen();

  check = digitalRead(start_IR_pin);

  if(check2){
    values();
  }

  if(check3){
    readpos();
  }
  
  

}