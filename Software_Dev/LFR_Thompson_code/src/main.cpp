#include <Arduino.h>
#include <BananaBar.h>
#include <InterCom.h>

SimpleCommand cmd;

const uint8_t led_ind = PB13;
Bar_Sensor_pinout barra_pinout={
  .mux_read_pin = PA3,
  .mux_selec_pins ={PA4,PA5,PA6,PA7},
  .Weighs_curve = {-80,-70,-60,-50,-40,-30,-20,-10,10,20,30,40,50,60,70,80}
};

bool check = false, check2 = false;
Bar_Sensors barra(&barra_pinout);

#define debug_port SerialUSB

void list(){
  cmd.list();
}
void calibrarBarra(){
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
}
void readpos(){
  int pos = barra.readPosition();
  debug_port.print("barra POS = ");
  debug_port.println(pos);
}
void doCheck(){
  check = !check;
}
void doCheck2(){
  check2 = !check2;
}
void printSensors(){
  barra.readSensors();
  for(uint8_t i=0;i<total_sensors;i++){
    debug_port.print((int)barra.sensors_values[i]);
    debug_port.print(", ");
  }
  debug_port.println(" ");
}
void printPOS(){
  debug_port.println(barra.readPosition(),2);
}
void setup(){
  barra.begin();
  pinMode(led_ind,OUTPUT);

  SerialUSB.begin(115200);

  Serial1.setTx(PB6);
  Serial1.setRx(PB7);
  Serial1.begin(9600);

  cmd.enable_echo(true);
  cmd.addCommand("list",list);
  cmd.addCommand("check",doCheck);
  cmd.addCommand("readpos",doCheck2);
  cmd.addCommand("cali",calibrarBarra);
  cmd.addCommand("r", readpos);
  cmd.begin(&debug_port);
  delay(250);
  //calibrarBarra();
}

void loop(){
  cmd.listen();
  if(check){
    printSensors();
  }
  if(check2){
    printPOS();
  }
}
