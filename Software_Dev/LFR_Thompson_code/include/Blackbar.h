/*
  BlackBar.h - Library for using NoMADA BlackBar.
  Created by Jonathan Y. Gabriel, July 6, 2019.
  Released into the public domain.
*/
#ifndef Blackbar_h
#define Blackbar_h
 
#include "Arduino.h"
 
class Blackbar
{
  public:
    Blackbar(int _s0,int _s1,int _s2,int _s3,int _out, int _line);
    int readSensor_raw(int num_sensor);
    void calibration();
    int readSensor(int s);
    int position();
    void printValues();
  private:
    int n=0;
    int p=0;
    int i=0;
    int j=0;
    int x=0;
    int s0=0;
    int s1=0;
    int s2=0;
    int s3=0;
    int out=0;
    int line=0;
    int S0, S1, S2, S3;
    int sensor[17]={};
    int valor_max[17]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    int valor_min[17]={1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023};
    int w=0,suma_w=0;
    float sum=0,e=0;
};
 
#endif