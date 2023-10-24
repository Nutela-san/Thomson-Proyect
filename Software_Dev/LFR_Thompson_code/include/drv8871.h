#ifndef DRV8871_H
#define DRV8871_H
#include <Arduino.h>

class drv8871
{
private:
    uint8_t INpins[2];
    int max_value = 255;
public:
    drv8871(uint8_t IN1, uint8_t IN2){
        INpins[0]=IN2;
        INpins[1]=IN1;
    }

    void begin(uint8_t resolution = 8, uint32_t frequency = 10000){
        max_value = pow(2,resolution) -1 ;
        for(uint8_t i = 0; i < 2; i++){
            pinMode(INpins[i], OUTPUT);
        }
        analogWriteResolution(resolution);
        analogWriteFrequency(frequency);
        sleep();
    }

    void writePWM(int driver_pwm){
        driver_pwm = constrain(driver_pwm,-max_value,max_value);

        if(driver_pwm > 0){
            analogWrite(INpins[0], driver_pwm);
            analogWrite(INpins[1], 0);
        }
        else if(driver_pwm < 0){
            analogWrite(INpins[1], abs(driver_pwm));
            analogWrite(INpins[0], 0);
        }
        else{
            analogWrite(INpins[0], max_value);
            analogWrite(INpins[1], max_value);
        }
    }

    void sleep(){
        analogWrite(INpins[0], 0);
        analogWrite(INpins[1], 0);
    }
};

#endif
