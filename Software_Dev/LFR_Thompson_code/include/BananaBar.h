#ifndef BANANABAR_H
#define BANANABAR_H
#include <Arduino.h>

float map_f(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#define total_sensors 16

struct Bar_Sensor_pinout{
  uint8_t mux_read_pin;       // {OUT SIGNAL}
  uint8_t mux_selec_pins[4];  // {S0,S1,S2,S3}
  float Weighs_curve[total_sensors];
};

class Bar_Sensors{
  public:
  Bar_Sensor_pinout *sensor_pins;
  float sensors_values[total_sensors]={0};
  uint16_t maxRangeCalibrated[total_sensors]={0};
  uint16_t minRangeCalibrated[total_sensors]={1024};
  float position;

  Bar_Sensors(Bar_Sensor_pinout *sensor_pins){
    this->sensor_pins = sensor_pins;
  }

  void begin(){
    pinMode(sensor_pins->mux_read_pin, INPUT);
    for(uint8_t i=0;i<4;i++){
      pinMode(sensor_pins->mux_selec_pins[i],OUTPUT);
    }
  }

  void readSensors(){
    uint8_t i;
    for(i=0;i<total_sensors;i++){
      (i & (1<<0)) ? digitalWrite(sensor_pins->mux_selec_pins[0],HIGH) : digitalWrite(sensor_pins->mux_selec_pins[0],LOW);
      (i & (1<<1)) ? digitalWrite(sensor_pins->mux_selec_pins[1],HIGH) : digitalWrite(sensor_pins->mux_selec_pins[1],LOW);
      (i & (1<<2)) ? digitalWrite(sensor_pins->mux_selec_pins[2],HIGH) : digitalWrite(sensor_pins->mux_selec_pins[2],LOW);
      (i & (1<<3)) ? digitalWrite(sensor_pins->mux_selec_pins[3],HIGH) : digitalWrite(sensor_pins->mux_selec_pins[3],LOW);
      sensors_values[i]= analogRead(sensor_pins->mux_read_pin);
    }
  }

  void doCalibration(){
    long j;
    uint8_t i;
    for(j=0;j<1000;j++){
      readSensors();
      for(i=0;i<total_sensors;i++){
        if(sensors_values[i]>maxRangeCalibrated[i]) maxRangeCalibrated[i] = sensors_values[i];
        if(sensors_values[i]<minRangeCalibrated[i]) minRangeCalibrated[i] = sensors_values[i];
      }
      delay(5);//si es necesario
    }
  }

  float readPosition(){
    uint8_t i;
    float  p=0.0f, w=0.0f;
    readSensors();
    for(i=0; i<total_sensors; i++){
      sensors_values[i] = constrain(sensors_values[i],minRangeCalibrated[i],maxRangeCalibrated[i]);
      sensors_values[i] = map_f(sensors_values[i],minRangeCalibrated[i],maxRangeCalibrated[i],1.0f,0.0f);
      p += sensors_values[i]*sensor_pins->Weighs_curve[i];
      w += sensors_values[i];  
    }

    if(w >= total_sensors*0.5f){ //CHECAR LA FUNCIONALIDAD
      w = 0.0f;
      p = 0.0f;
      for(i=0;i<total_sensors;i++){
        sensors_values[i] = 1.0f - sensors_values[i];
        p += sensors_values[i]*sensor_pins->Weighs_curve[i];
        w += sensors_values[i]; 
      }
    } 
    (w>0.0f)?position = (p/w): position = position;
  
    return position;
  }

};

#endif