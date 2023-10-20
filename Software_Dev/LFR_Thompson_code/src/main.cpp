#include <Arduino.h>
#include <TB6612.h>
#include <InterCom.h>
#include <CMSIS_DSP.h>

const uint8_t led_ind =PC13,  boton = PB0;
//const uint8_t led_ind = PB13, starter_pin = PB12;
//const uint8_t m_der_pin[2] = {PB3,PB5}, m_izq_pin[2] = {PA15,PA10};

//drv8871 motorIzq(m_izq_pin[0],m_izq_pin[1]);
//drv8871 motorDer(m_der_pin[0],m_der_pin[1]);
//uint8_t enc_A_pin[2] = {PA8,PA9}, enc_B_pin[2] = {PB8,PB9};
        //encoder A = motorIZQ    encoderB = derecho
uint8_t enc_A_pin[2] = {PA8,PA9}, enc_B_pin[2] = {PB9,PB8};

TB6612_pinout motors_pinout = {
  .PWMA_pin = PA10,
  .AIN2_pin = PA11,
  .AIN1_pin = PA12,
  .STBY_pin = PA15,
  .BIN1_pin = PB3,
  .BIN2_pin = PB4,
  .PWMB_pin = PB5
};
MotorServo_TB driver_motores(motors_pinout,enc_A_pin,enc_B_pin);
unsigned int steps_per_rev = 36;
/*
Bar_Sensor_pinout barra_pinout={
  .mux_read_pin = PA3,
  .mux_selec_pins ={PA4,PA5,PA6,PA7},
  .Weighs_curve = {-80,-70,-60,-50,-40,-30,-20,-10,10,20,30,40,50,60,70,80}
};
Bar_Sensors barra(&barra_pinout);
*/

uint16_t vs[8] ={0};

SimpleCommand cmd;

HardwareTimer *pid_INT = new HardwareTimer(TIM10);
const int hz_control= 100;

arm_pid_instance_f32 pid_parametes;
arm_pid_instance_f32 pid_Motor_A_parametes;
arm_pid_instance_f32 pid_Motor_B_parametes;
volatile float vsetpoint_a = 0;
volatile float vsetpoint_b = 0;
float setpoint_a = 0;
float setpoint_b = 0;
volatile float vel_A, vel_B;

bool check = false, check2 = false;
float setpoint = 30;

#define debug_port Serial1

void config_barra(){
  //barra.setTypeAnalog();
  //barra.setSensorPins((const uint8_t[]){PA0, PA1, PA2, PA3, PA4, PA5,PA6,PA7}, 8);
  //barra.setEmitterPin(PC15);
}

void list(){
  cmd.list();
}

void calibrarBarra(){

  //for(uint8_t i = 0; i<100; i++){
  //  barra.calibrate();
  //  delay(20);
  //}
  //digitalWrite(led_ind,HIGH);
  //digitalWrite(led_ind,LOW);
  
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
  //int pos = barra.readLineWhite(vs);//barra.readPosition();
  //debug_port.print("barra POS = ");
  //debug_port.println(pos);
}

void doCheck(){
  check = !check;
}

void doCheck2(){
  check2 = !check2;
}

void refresh(){
  arm_pid_init_f32(&pid_Motor_A_parametes, 1);
  arm_pid_init_f32(&pid_Motor_B_parametes, 1);
}

void config_commands(){
  cmd.enable_echo(true);
  cmd.addCommand("list",list);
  cmd.addCommand("check2",doCheck2);
  cmd.addCommand("check",doCheck);
  cmd.addCommand("refresh", refresh);
  cmd.addCommand("cali",calibrarBarra);
  cmd.addCommand("pa",&pid_Motor_A_parametes.Kp);
  cmd.addCommand("ia",&pid_Motor_A_parametes.Ki);
  cmd.addCommand("da",&pid_Motor_A_parametes.Kd);
  cmd.addCommand("pb",&pid_Motor_B_parametes.Kp);
  cmd.addCommand("ib",&pid_Motor_B_parametes.Ki);
  cmd.addCommand("db",&pid_Motor_B_parametes.Kd);
  cmd.addCommand("s",&setpoint);
  cmd.begin(&debug_port);
}

void PID_ISR(){
  vel_A = driver_motores.readVel_A();
  vel_B = driver_motores.readVel_B();

  if(check){
    float errorA = vsetpoint_a - vel_A; 
    float errorB = vsetpoint_b - vel_B; 

    int pwmA = arm_pid_f32(&pid_Motor_A_parametes,errorA);
    int pwmB = arm_pid_f32(&pid_Motor_B_parametes,errorB);
    driver_motores.setPWM(pwmA,pwmB);
  }
  else{
    driver_motores.setPWM(0,0);
  }

  /*
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
  */
}

void ISR_encA(){
  if(digitalRead(driver_motores.encA_B_pin)){
    driver_motores.enc_steps_A--;
  }
  else{
    driver_motores.enc_steps_A++;
  }
  //debug_port.println(driver_motores.enc_steps_A);
}

void ISR_encB(){
  if(digitalRead(driver_motores.encB_B_pin)){
    driver_motores.enc_steps_B--;
  }

  else{
    driver_motores.enc_steps_B++;
  }
  //debug_port.println(driver_motores.enc_steps_B);
}

void config_timer_interrup(){
  
  pid_INT->pause();
  pid_INT->setMode(1,TIMER_DISABLED);   //Config para que solo sirva de contador para interrupt
  pid_INT->setOverflow((uint32_t)hz_control,HERTZ_FORMAT); //Interrupt 1 veces por segundo.
  pid_INT->attachInterrupt(1,PID_ISR);

  pid_INT->refresh();

}

void config_PID(){
  /*
  pid_parametes.Kp = 3.0f;
  pid_parametes.Ki = 0.0f;
  pid_parametes.Kd = 0.5f;
  arm_pid_init_f32(&pid_parametes,(int32_t)1);*/

  //contstantes para s = 50 rad/s sin turbina
  pid_Motor_A_parametes.Kp = 4.9f;
  pid_Motor_A_parametes.Ki = 2.5f;
  pid_Motor_A_parametes.Kd = 0.02f;
  arm_pid_init_f32(&pid_Motor_A_parametes,1);

  pid_Motor_B_parametes.Kp = 4.0f;
  pid_Motor_B_parametes.Ki = 1.5f;
  pid_Motor_B_parametes.Kd = 0.01f;
  arm_pid_init_f32(&pid_Motor_B_parametes,1);
  
}

void setup(){

  //---Inicializacion de pines---
  pinMode(led_ind,OUTPUT);
  //pinMode(starter_pin,INPUT);
  pinMode(boton,INPUT_PULLUP);


  //barra.begin();        //init de barra de sensores
  config_barra();
  driver_motores.begin(hz_control,steps_per_rev,ISR_encA,ISR_encB);
  //motorDer.begin(10);   //init de drives para motores DRV8871
  //motorIzq.begin(10);
  
  //---Inicializacion de protocolos de comunicacion---
  SerialUSB.begin(115200);  

  Serial1.setTx(PB6);
  Serial1.setRx(PB7);
  Serial1.begin(9600);

  //---Configuracion de la logica---
  config_commands();
  config_timer_interrup();
  config_PID();

  pid_INT->resume();  //Iniciando el TIMER para la interupcion del PID
}

void loop(){
  cmd.listen();
  if(check2){
    //readpos();
    debug_port.print("wA = ");
    debug_port.print(vel_A,3);
    debug_port.print(",w = ");
    debug_port.println(vel_B,3);
    delay(100);
  }
  vsetpoint_a = setpoint;
  vsetpoint_b = setpoint;
}
