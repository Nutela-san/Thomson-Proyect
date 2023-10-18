#include "AdvancePID.h"

void AdvancePID_begin(PID_Parameters *pid, float _kp=1, float _ki=0, float _kd=0,float _tau=0.0f, float _Ts=0.01f){
  pid->controller_out = 0.0f;
  pid->last_error = 0.0f;
  pid->last_measurment = 0.0f;
  pid->r_derivative = 0.0f;
  pid->r_integral = 0.0f;

  pid->Kp =_kp;
  pid->Ki =_ki;
  pid->Kd =_kd;
  
  pid->tau = _tau;
  pid->Ts =_Ts;
}

float AdvancePID_Update(PID_Parameters *pid, float measurment, float setpoint){
  float error = setpoint - measurment;

  float r_proportional = pid->Kp * error;

  pid->r_integral = pid->r_integral + 0.5f*pid->Ki*pid->Ts*(error + pid->last_error);

  float range_integral[2];
  (pid->range_output[1] > r_proportional)? range_integral[1] = pid->range_output[1] - r_proportional:  range_integral[1] = 0.0f;
  (pid->range_output[0] < r_proportional)? range_integral[0] = pid->range_output[0] - r_proportional:  range_integral[0] = 0.0f;

  pid->r_integral = constrain(pid->r_integral, range_integral[0], range_integral[1]);

  pid->r_derivative = (2.0f * pid->Kd * (measurment - pid->last_measurment) 
                    + (2.0f*pid->tau -pid->Ts)* pid->r_derivative)
                    / (2.0f * pid->tau +pid->Ts);

  pid->controller_out = r_proportional + pid->r_integral + pid->r_derivative;
  pid->controller_out = constrain(pid->controller_out, pid->range_output[0], pid->range_output[1]);
  pid->last_error = error;
  pid->last_measurment = measurment;
  return pid->controller_out;
}

/*
AdvancePID::AdvancePID(){};

#ifdef __AVR_ATmega328P__

void AdvancePID::begin(unsigned int SampleTime_m, uint8_t TIMER_2_USED){
    TIMER_2_USED = constrain(TIMER_2_USED,0,2);
    cli();
    */
    /*
    *   Timer mode = CTC 
    * Pines TIMER0:
    *   OC0A = PD6 = 6(Arduino)
    *   OC0B = PD5 = 5  (Arduino)
    * Pines TIMER1:
    *   OC1A = PB1 = 9(Arduino)
    *   OC1B = Pb2 = 10(Arduino)
    * Pines TIMER2:
    *   OC2A = PB3 = 11(Arduino)
    *   OC2B = PD3 = 3(Arduino)
    * 
    * f_ocnx = (f_clk_io)/(2*N*(1+OCRnX));
    * OCRnX =  ((f_clk_io)/(2*N*f_ocnx))-1;
    * 
    * Referencia de 1ms para TIMER0 y TIMER2:
    *     freq_PID = 1000Hz , f_clk_io =20MHz, N = 64, OCRnX = 155.25 = 155
    *     freq_PID = 1000Hz , f_clk_io =16MHz, N = 64, OCRnX = 124
    *     freq_PID = 1000Hz , f_clk_io = 8MHz, N = 64, OCRnX = 61.5 = 62
    *     freq_PID = 1000Hz , f_clk_io = 1MHz,  N = 8, OCRnX = 61.5 = 62
    * 
    * Referencia de 1ms para TIMER1:
    *     freq_PID = 1000Hz , f_clk_io =20MHz, N =  8, OCR1X = 1249
    *     freq_PID = 1000Hz , f_clk_io =16MHz, N =  1, OCR1X = 7999
    *     freq_PID = 1000Hz , f_clk_io = 8MHz, N =  1, OCR1X = 3999
    *     freq_PID = 1000Hz , f_clk_io = 1MHz,  N = 1, OCR1X = 499
    * 
    */
    // Configurar el temporizador 
    /*
    switch (TIMER_2_USED){ 
    case 0:{

      TCCR0A = (1<<WGM01);//TIMER in CTC mode
      TCNT0 = 0; //clear counter
      TIMSK0 = (1<<OCIE0A); // enable_interrupt;
      TCCR0B = (0<<CS02)|(0<<CS01)|(0<<CS00); //N=0 -> TIMER STOPPED
      #if F_CPU == 20000000UL
        time_scaler_register = (0<<CS02)|(1<<CS01)|(1<<CS00);//N = 64
        OCR0A = 155;
      #elif F_CPU == 16000000UL
        time_scaler_register = (0<<CS02)|(1<<CS01)|(1<<CS00);//N = 64
        OCR0A = 124;
      #elif F_CPU == 8000000UL
        time_scaler_register = (0<<CS02)|(1<<CS01)|(1<<CS00);//N = 64
        OCR0A = 62;
      #elif F_CPU == 1000000UL
        time_scaler_register = (0<<CS02)|(1<<CS01)|(0<<CS00);//N = 8
        OCR0A = 62;
      #endif
      break;
    }
    case 1:{
      TCCR1A = (1<<WGM12); //CTC mode TOP =OCR1A
      TCNT1 = 0; //clear counter
      TIMSK1 = (1<<OCIE1A); // enable interrupt
      TCCR1B = (0<<CS12)|(0<<CS11)|(0<<CS10); //N=0 -> TIMER STOPPED
      #if F_CPU == 20000000UL
        time_scaler_register = (0<<CS12)|(1<<CS11)|(0<<CS10);//N = 8
        OCR1A = 1249;
      #elif F_CPU == 16000000UL
        time_scaler_register = (0<<CS12)|(0<<CS11)|(1<<CS10);//N = 1
        OCR1A = 7999;
      #elif F_CPU == 8000000UL
        time_scaler_register = (0<<CS12)|(0<<CS11)|(1<<CS10);//N = 1
        OCR1A = 3999;
      #elif F_CPU == 1000000UL
        time_scaler_register = (0<<CS12)|(0<<CS11)|(1<<CS10);//N = 1
        OCR1A = 499;
      #endif
      break;
    }
    case 2:{
      TCCR2A = (1<<WGM21);//TIMER in CTC mode
      TCNT2 = 0; //clear counter
      TIMSK2 = (1<<OCIE2A); // enable_interrupt;
      TCCR2B = (0<<CS22)|(0<<CS21)|(0<<CS20); //N=0 -> TIMER STOPPED
      #if F_CPU == 20000000UL
        time_scaler_register = (0<<CS20)|(1<<CS21)|(1<<CS20);//N = 64
        OCR2A = 155;
      #elif F_CPU == 16000000UL
        time_scaler_register = (0<<CS22)|(1<<CS21)|(1<<CS20);//N = 64
        OCR2A = 124;
      #elif F_CPU == 8000000UL
        time_scaler_register = (0<<CS22)|(1<<CS21)|(1<<CS20);//N = 64
        OCR2A = 62;
      #elif F_CPU == 1000000UL
        time_scaler_register = (0<<CS22)|(1<<CS21)|(0<<CS20);//N = 8
        OCR2A = 62;
      #endif
      break;
    }
    default:
      //config error.
      break;
    }
    // Habilitar interrupciones globales
    sei();
}

#else 
  //NOT IMPLEMENTED YET
  void AdvancePID::begin(unsigned int SampleTime_m){

  }
#endif

void AdvancePID::setGains(float kp, float ki, float kd){
  this->Kp = kp;
  this->Ki = ki;
  this->Kd = kd;
}

void AdvancePID::setOutLimits(float r_min, float r_max){
  this->rang_out[0] = r_min;
  this->rang_out[1] = r_max;
}

void AdvancePID::setOutLimits(float r){
  this->setOutLimits(-r,r);
}

void AdvancePID::setLPFparameter(float alpha){
  this->alpha =alpha;
}

void AdvancePID::calulate_out(float input, float setpoint){
  float error = setpoint - input;
  
  float r_prop = Kp*error;
  
  r_int = r_int + 0.5f*Ki*Ts*(error+last_e);

  (rang_out[1]>r_prop)? rang_integral[1] = rang_out[1] - r_prop: rang_integral[1] =0.0f;
  (rang_out[0]<r_prop)? rang_integral[0] = rang_out[0] - r_prop: rang_integral[0] =0.0f;
  
  r_int = constrain(r_int,rang_integral[0],rang_integral[1]);

  r_dev = (2.0f* Kd * (input-last_input) + (2.0f * alpha - Ts)* r_dev)
          +(2.0f*alpha + Ts);

  pid_out = r_prop + r_int + r_dev;
  pid_out = constrain(pid_out,rang_out[0],rang_out[1]);

  last_input = input;
  last_e = error;

}

#ifdef __AVR_ATmega328P__
bool Rissing =false;

ISR(TIMER0_COMPA_vect){
  
}

ISR(TIMER1_COMPA_vect){

}

ISR(TIMER2_COMPA_vect){

}
#else
#endif
*/