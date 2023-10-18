#ifndef _ADVANCEPID_H_
#define _ADVANCEPID_H_

#include <Arduino.h>

struct PID_Parameters
{
  float Kp;   //Controller Gains
  float Ki;
  float Kd;

  float tau;  //Low Pass Filter(LPF) derivative time constant
  float range_output[2];//limit controller output range
  float Ts;   //sample time

  // Controller MEMORY
  float r_integral;
  float r_derivative;
  float last_error;
  float last_measurment;

  float controller_out;
};

void AdvancePID_begin(PID_Parameters *pid,float _kp, float _ki, float _kd,float _tau, float _Ts);
float AdvancePID_Update(PID_Parameters *pid, float measurment, float setpoint);


/*
class AdvancePID{
  private:
    float r_int, r_dev, last_e, last_input;
    float rang_integral[2], rang_out[2];

    float Ts; //sampletime
    bool enable_LPF = false;
    uint8_t time_scaler_register =0;
    
  public:
    float Kp=0, Kd=0, Ki=0;
    float alpha; //para ajustar el filtro pasa bajas
    float pid_out=0;
    AdvancePID();

    #ifdef __AVR_ATmega328P__
      void begin(unsigned int SampleTime_ms , uint8_t TIMER_2_USED = 1);
    #else
      void begin(unsigned int SampleTime_m);
    #endif

    void setGains(float kp, float ki, float kd);
    void setOutLimits(float r_min, float r_max);
    void setOutLimits(float r);
    void setLPFparameter(float alpha);
    void calulate_out(float input, float setpoint);

};
*/
#endif