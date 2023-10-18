#include <Arduino.h>
//#include <BananaBar.h>
//#include <drv8871.h>
#include <QTRSensors.h>
#include <TB6612.h>
#include <InterCom.h>
#include <CMSIS_DSP.h>

const uint8_t led_ind =PC13,  boton = PB0;
//const uint8_t led_ind = PB13, starter_pin = PB12;
//const uint8_t m_der_pin[2] = {PB3,PB5}, m_izq_pin[2] = {PA15,PA10};

//drv8871 motorIzq(m_izq_pin[0],m_izq_pin[1]);
//drv8871 motorDer(m_der_pin[0],m_der_pin[1]);
TB6612_pinout motors_pinout = {
  .PWMA_pin = PA11,
  .AIN2_pin = PA1,
  .AIN1_pin = PB5,
  .STBY_pin = PB6,
  .BIN1_pin = PB7,
  .BIN2_pin = PB8,
  .PWMB_pin = PB9
};
TB6612 driver_motores(motors_pinout);
/*
Bar_Sensor_pinout barra_pinout={
  .mux_read_pin = PA3,
  .mux_selec_pins ={PA4,PA5,PA6,PA7},
  .Weighs_curve = {-80,-70,-60,-50,-40,-30,-20,-10,10,20,30,40,50,60,70,80}
};
Bar_Sensors barra(&barra_pinout);
*/
QTRSensors barra;
uint16_t vs[8] ={0};

SimpleCommand cmd;

HardwareTimer *pid_INT = new HardwareTimer(TIM10);
const int hz_control= 200;

arm_pid_instance_f32 pid_parametes;

bool check = false, check2 = false;
float setpoint= 65;

#define debug_port Serial1

void config_barra(){
  barra.setTypeAnalog();
  barra.setSensorPins((const uint8_t[]){PA0, PA1, PA2, PA3, PA4, PA5,PA6,PA7}, 8);
  barra.setEmitterPin(PC15);
}

void list(){
  cmd.list();
}

void calibrarBarra(){

  digitalWrite(led_ind,HIGH);
  for(uint8_t i = 0; i<100; i++){
    barra.calibrate();
    delay(20);
  }
  digitalWrite(led_ind,LOW);
  
  /*
  digitalWrite(led_ind,HIGH);
  barra.doCalibration();
  digitalWrite(led_ind,LOW);
  debug_port.println("Calibraccion completatada");
  for(uint8_t i=0;i<total_sensors;i++){
    debug_port.print("S");
    debug_port.print(i);
    debug_port.print(" = {");
    debug_port.print(barra.minRangeCalibrated[i]);
    debug_port.print(",");
    debug_port.print(barra.maxRangeCalibrated[i]);
    debug_port.print("},\t");
  }
  debug_port.println(" ");
  */
  
}

void readpos(){
  int pos = barra.readLineWhite(vs);//barra.readPosition();
  debug_port.print("barra POS = ");
  debug_port.println(pos);
}

void doCheck(){
  check = !check;
}

void doCheck2(){
  check2 = !check2;
}

void config_commands(){
  cmd.enable_echo(true);
  cmd.addCommand("list",list);
  cmd.addCommand("check",doCheck);
  cmd.addCommand("readpos",doCheck2);
  cmd.addCommand("cali",calibrarBarra);
  cmd.addCommand("p",&pid_parametes.Kp);
  cmd.addCommand("i",&pid_parametes.Ki);
  cmd.addCommand("d",&pid_parametes.Kd);
  cmd.addCommand("s",&setpoint);
  cmd.begin(&debug_port);
}

void PID_ISR(){
  //debug_port.printf("HOLA soy la interrupcion de %d hz \n",1);
  //unsigned long last_t = micros();   
  //float T;
  if(check){
    //float error = -barra.readPosition();
    float error = 35.0f-((float)barra.readLineWhite(vs)/100.0f);
    float inc_pwm = arm_pid_f32(&pid_parametes,error);
    //motorDer.writePWM((int)(setpoint - inc_pwm));
    //motorIzq.writePWM((int)(setpoint + inc_pwm));
    driver_motores.setPWM((int)(setpoint + inc_pwm),(int)(setpoint - inc_pwm));
  }
  else{
    //motorDer.writePWM(0);
    //motorIzq.writePWM(0);
    driver_motores.setPWM(0,0);
  }

  //T = (float)(micros() -last_t);
  /*
  debug_port.println("--Control--");
  debug_port.print("setpoint:");
  debug_port.print(setpoint,3);
  debug_port.print(", medicion:");
  debug_port.print(medicion,3);
  debug_port.print(", OUT_C:");
  debug_port.print(out_control,3);
  //debug_port.print(", T_calulo:");
  //debug_port.print(T,3);
  debug_port.print(", hz_C:");
  debug_port.println(hz_control);
  */

}

void config_timer_interrup(){
  
  pid_INT->pause();
  pid_INT->setMode(1,TIMER_DISABLED);   //Config para que solo sirva de contador para interrupt
  pid_INT->setOverflow((uint32_t)hz_control,HERTZ_FORMAT); //Interrupt 1 veces por segundo.
  pid_INT->attachInterrupt(1,PID_ISR);

  pid_INT->refresh();

}

void config_PID(){
  pid_parametes.Kp = 3.0f;
  pid_parametes.Ki = 0.0f;
  pid_parametes.Kd = 0.5f;
  arm_pid_init_f32(&pid_parametes,(int32_t)1);
}

void setup(){

  //---Inicializacion de pines---
  pinMode(led_ind,OUTPUT);
  //pinMode(starter_pin,INPUT);
  pinMode(boton,INPUT_PULLUP);


  //barra.begin();        //init de barra de sensores
  config_barra();
  driver_motores.begin();
  //motorDer.begin(10);   //init de drives para motores DRV8871
  //motorIzq.begin(10);
  
  //---Inicializacion de protocolos de comunicacion---
  SerialUSB.begin(115200);  

  //Serial1.setTx(PB6);
  //Serial1.setRx(PB7);
  Serial1.begin(9600);

  //---Configuracion de la logica---
  config_commands();
  config_timer_interrup();
  config_PID();

  //while(digitalRead(boton)){
  //  cmd.listen();
  //}
  //calibrarBarra();

  pid_INT->resume();  //Iniciando el TIMER para la interupcion del PID
}

void loop(){
  cmd.listen();
  if(check2){
    readpos();
  }

}
