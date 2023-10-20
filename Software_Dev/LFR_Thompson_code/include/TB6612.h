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
  
  const int max_pwm_value = 2.0E8 - 1;

protected:
  TB6612_pinout pins ;

public:
  TB6612(uint8_t STBY,uint8_t PWMA, uint8_t AIN1, uint8_t AIN2,uint8_t PWMB, uint8_t BIN1, uint8_t BIN2);
  TB6612(TB6612_pinout pins);
  TB6612(){}
  
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


class MotorServo_TB : public TB6612{
  public:
    uint8_t encA_A_pin, encA_B_pin, encB_A_pin, encB_B_pin; //pienes de encoder;
    unsigned int  steps_per_rev; //resolucion de encoder
    float steps2rad;    //Factor de conversion;
    float steps2rad_S;    //Factor de conversion;
    
    float Ts;         //(S)Tiempo de sampleo para medir velocidad


    long enc_steps_A = 0, enc_steps_B = 0;  //contador de encoder (medir posicion)
    long last_enc_steps_A = 0, last_enc_steps_B = 0;  //contador de encoder (medir posicion)

    MotorServo_TB(TB6612_pinout pinout, uint8_t enc_A[2], uint8_t enc_B[2]);

    void begin(float Ts,unsigned int steps_per_rev,void(*irs_encoderA)(void),void(*irs_encoderB)(void));

    float readPos_A();
    float readVel_A();
    float readPos_B();
    float readVel_B();

    ~MotorServo_TB(){}//destructor

};


MotorServo_TB::MotorServo_TB(TB6612_pinout pinout, uint8_t enc_A[2], uint8_t enc_B[2]){
  this->pins = pinout;
  this->encA_A_pin = enc_A[0];
  this->encA_B_pin = enc_A[1];
  this->encB_A_pin = enc_B[0];
  this->encB_B_pin = enc_B[1];
}

void MotorServo_TB::begin(float freq,unsigned int steps_per_rev,void(*irs_encoderA)(void),void(*irs_encoderB)(void)){
  TB6612::begin();
  pinMode(encA_A_pin,INPUT);
  pinMode(encA_B_pin,INPUT);
  pinMode(encB_A_pin,INPUT);
  pinMode(encB_B_pin,INPUT);

  attachInterrupt(digitalPinToInterrupt(encA_A_pin),irs_encoderA,RISING);
  attachInterrupt(digitalPinToInterrupt(encB_A_pin),irs_encoderB,RISING);

  this->steps_per_rev = steps_per_rev;
  this->Ts = 1.0f/freq;

  this->steps2rad = 2*3.1416f/(float)this->steps_per_rev;
  this->steps2rad_S = steps2rad / this->Ts;
}

float MotorServo_TB::readPos_A(){
  return enc_steps_A*steps2rad;
}

float MotorServo_TB::readVel_A(){
  float vel = (enc_steps_A-last_enc_steps_A)*steps2rad_S;
  last_enc_steps_A = enc_steps_A;
  return vel;
  
}

float MotorServo_TB::readPos_B(){
  return enc_steps_B*steps2rad;
}

float MotorServo_TB::readVel_B(){
  float vel = (enc_steps_B-last_enc_steps_B)*steps2rad_S;
  last_enc_steps_B = enc_steps_B;
  return vel;
}

#endif