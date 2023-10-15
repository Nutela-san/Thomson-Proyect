#ifndef TT6612_H
#define TT6612_H
#include <Arduino.h>

struct TB6612_pinout
{
  uint8_t PWMA_pin;
  uint8_t AIN2_pin;
  uint8_t AIN1_pin;
  uint8_t STBY_pin;
  uint8_t BIN1_pin;
  uint8_t BIN2_pin;
  uint8_t PWMB_pin;
};

class TB6612{
private:
  /* data */
  TB6612_pinout pins ;
  const int max_pwm_value = 2.0E8 - 1;

public:
  TB6612(uint8_t STBY,uint8_t PWMA, uint8_t AIN1, uint8_t AIN2,uint8_t PWMB, uint8_t BIN1, uint8_t BIN2);
  TB6612(TB6612_pinout pins);

  void begin();
  void enableDriver(bool en =true);
  void setPWM(int pwmA,int pwmB);

  ~TB6612();
};

TB6612::TB6612(uint8_t STBY,uint8_t PWMA, uint8_t AIN1, uint8_t AIN2,uint8_t PWMB, uint8_t BIN1, uint8_t BIN2){
  this->pins.STBY_pin = STBY;
  this->pins.PWMA_pin = PWMA;
  this->pins.AIN1_pin = AIN1;
  this->pins.AIN2_pin = AIN2;
  this->pins.PWMA_pin = PWMB;
  this->pins.AIN1_pin = BIN1;
  this->pins.AIN2_pin = BIN2;
}

TB6612::TB6612(TB6612_pinout pins){
  this->pins = pins;
}

TB6612::~TB6612(){}

void TB6612::begin(){
  pinMode(pins.PWMA_pin,OUTPUT);
  pinMode(pins.PWMB_pin,OUTPUT);
  pinMode(pins.AIN1_pin,OUTPUT);
  pinMode(pins.AIN2_pin,OUTPUT);
  pinMode(pins.BIN1_pin,OUTPUT);
  pinMode(pins.BIN2_pin,OUTPUT);
  pinMode(pins.STBY_pin,OUTPUT);

  setPWM(0,0);

  enableDriver();
}

void TB6612::enableDriver(bool en){
  digitalWrite(pins.STBY_pin,en);
}

void TB6612::setPWM(int pwmA,int pwmB){
  pwmA = constrain((int)pwmA,-max_pwm_value,max_pwm_value);
  pwmB = constrain((int)pwmB,-max_pwm_value,max_pwm_value);

  if(pwmA >0){
    digitalWrite(pins.AIN1_pin,HIGH);
    digitalWrite(pins.AIN2_pin,LOW);
  }
  else if(pwmA<0){
    digitalWrite(pins.AIN2_pin,HIGH);
    digitalWrite(pins.AIN1_pin,LOW);
    pwmA = abs(pwmA);
  }
  else{
    digitalWrite(pins.AIN1_pin,LOW);
    digitalWrite(pins.AIN2_pin,LOW);
  }

  if(pwmB>0){
    digitalWrite(pins.BIN1_pin,HIGH);
    digitalWrite(pins.BIN2_pin,LOW);
  }
  else if(pwmB<0){
    digitalWrite(pins.BIN2_pin,HIGH);
    digitalWrite(pins.BIN1_pin,LOW);
    pwmB = abs(pwmB);
  }
  else{
    digitalWrite(pins.BIN1_pin,LOW);
    digitalWrite(pins.BIN2_pin,LOW);
  }

  analogWrite(pins.PWMA_pin,pwmA);
  analogWrite(pins.PWMB_pin,pwmB);
}

#endif