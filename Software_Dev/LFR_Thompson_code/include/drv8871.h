#ifndef DRV8871_H
#define DRV8871_H
#include <Arduino.h>

class drv8871
{
private:
    uint8_t INpins[2];
    unsigned int max_value;
public:
    drv8871(uint8_t INpins[2]){
        for(uint8_t i = 0; i < 2; i++){
            this->INpins[i] = INpins[i];
        }
    }

    void begin(uint8_t resolution = 10, uint32_t frequency = 10000){
        max_value = pow(2, resolution) - 1;

        for(uint8_t i = 0; i < 2; i++){
            pinMode(INpins[i], OUTPUT);
        }

        analogWriteResolution(resolution);
        analogWriteFrequency(frequency);
    }

    void writePWM(int driver_pwm){
        driver_pwm = constrain(driver_pwm, -max_value, max_value);

        if(driver_pwm > 0){
            analogWrite(INpins[0], driver_pwm);
            digitalWrite(INpins[1], 0);
        }
        else if(driver_pwm < 0){
            analogWrite(INpins[1], abs(driver_pwm));
            digitalWrite(INpins[0], 0);
        }
        else{
            digitalWrite(INpins[0], HIGH);
            digitalWrite(INpins[1], HIGH);
        }
    }
};

#endif
