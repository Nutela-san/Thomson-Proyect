#ifndef SIMPLEBARSENSOR_H
#define SIMPLEBARSENSOR_H
#include <Arduino.h>

template<size_t num_sensors = 8>
class SimpleBarSensor{
public:
  //BarSensor_parameters<> *parameters; /* data */
  const uint8_t num_of_sensors = num_sensors;
  const float pos_error_code = 35505.0f;
  uint8_t sensor_pins[num_sensors];
  uint8_t enable_pin;
  int w[num_sensors] = {0};
  float sensors_values[num_sensors]={0};
  bool sensor_values_boolean[num_sensors]={false};
  uint16_t maxRangeCalibrated[num_sensors]={0};
  uint16_t minRangeCalibrated[num_sensors]={1024};
  uint16_t midpointRangeCalibrated[num_sensors];
  float position;


  bool white_line = false;
  bool enable_state = true;
  bool isCalibrated = false;
  bool doDigitalConvertion = false;

  SimpleBarSensor(const uint8_t sensor_pins[num_sensors] ,uint8_t enable_pin ,const int w[num_sensors], bool detecting_WhiteLine = false){
    for(uint8_t i = 0 ; i<num_of_sensors; i++){
      this->sensor_pins[i] = sensor_pins[i];
      this->w[i] = w[i];
    }
    this->enable_pin = enable_pin;
    white_line = detecting_WhiteLine;
  }
  ~SimpleBarSensor(){}

  void begin(bool enable_state = true){
    for(uint8_t i=0;i<num_of_sensors;i++){
      pinMode(sensor_pins[i],INPUT);
    }
    pinMode(enable_pin,OUTPUT);
    this->enable_state = enable_state;
    digitalWrite(enable_pin,!this->enable_state);
  }

  void enable(bool en){
    if(en){
      digitalWrite(enable_pin, enable_state);
    }
    else{
      digitalWrite(enable_pin, !enable_state);
    }
  }

  void digitalConvetion(bool en){
    doDigitalConvertion = en;
  }

  void readSensors(){
    uint8_t i;
    enable(true);
    for(i=0;i<num_of_sensors;i++){
      sensors_values[i] = analogRead(sensor_pins[i]);
      if(isCalibrated && doDigitalConvertion){
        float p = map_f(sensors_values[i], minRangeCalibrated[i], maxRangeCalibrated[i],
                        (float)(white_line), (float)(!white_line));
        (p>=0.5)? sensor_values_boolean[i] = true: sensor_values_boolean[i] = false;
      }
    }
    enable(false);
  }

  float readPosition(){
    if(isCalibrated){
      uint8_t i;
      float  p=0.0f, w=0.0f;
      readSensors();
      for(i=0; i<num_of_sensors; i++){
        sensors_values[i] = constrain(sensors_values[i], minRangeCalibrated[i], maxRangeCalibrated[i]);
        sensors_values[i] = map_f(sensors_values[i], minRangeCalibrated[i], maxRangeCalibrated[i],
                                  (float)(white_line), (float)(!white_line));
        p += sensors_values[i]*this->w[i];
        w += sensors_values[i];  
      }


      (w>0.2f)?position = (p/w): position = position;
    
      return position;
    }
    else{
      return pos_error_code; //error code, need calibration
    }
  }
  
  void doCalibration(){
    long j;
    uint8_t i;
    for(j=0;j<10;j++){
      readSensors();
      for(i=0;i<num_of_sensors;i++){
        if(sensors_values[i]>maxRangeCalibrated[i]) maxRangeCalibrated[i] = sensors_values[i];
        if(sensors_values[i]<minRangeCalibrated[i]) minRangeCalibrated[i] = sensors_values[i];
      }
      delay(5);//si es necesario
    }
    isCalibrated = true;
  }

  float map_f(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
};

#endif