/*
  BlackBar.h - Library for using NoMADA BlackBar.
  Created by Jonathan Y. Gabriel, July 6, 2019.
  Released into the public domain.
*/
 
#include "Arduino.h"
#include "Blackbar.h"
 
Blackbar::Blackbar(int _s0,int _s1,int _s2,int _s3,int _out, int _line)
{
  pinMode(_s0,OUTPUT);
  pinMode(_s1,OUTPUT);
  pinMode(_s2,OUTPUT);
  pinMode(_s3,OUTPUT);
  s0=_s0;
  s1=_s1;
  s2=_s2;
  s3=_s3;
  out=_out;
  line=_line;
  
}
 
int Blackbar::readSensor_raw(int num_sensor)
   {
      S0 = num_sensor & B00000001 ;  digitalWrite ( s0 , S0) ; //Solo quiero el ultimo bit
      S1 = ( num_sensor >> 1 ) & B00000001 ; digitalWrite ( s1 , S1) ;
      S2 = ( num_sensor >> 2 ) & B00000001 ; digitalWrite (s2 , S2) ;
      S3 = ( num_sensor >> 3 ) & B00000001 ; digitalWrite (s3 , S3) ;
      return analogRead(out);      
   }

void Blackbar::calibration()
   {
    if(line==0){n=0;p=255;}
    else {n=255;p=0;}
    for(j=0;j<3500;j++)
    {
      for (i=0;i<16;i++)
       {
        sensor[i]=readSensor_raw(i);
        if(sensor[i]>valor_max[i])valor_max[i]=sensor[i];
        if(sensor[i]<valor_min[i])valor_min[i]=sensor[i];
       }
    }    
   }

 int Blackbar::readSensor(int s)
{
    sensor[s]=map(readSensor_raw(s),valor_min[s],valor_max[s],n,p);
    sensor[s]=constrain(sensor[s],0,255);
    return sensor[s];
}

int Blackbar::position()
{
  suma_w=0;
  sum=0;
  for(i=0;i<16;i++)
  {
    sensor[i]=map(readSensor_raw(i),valor_min[i],valor_max[i],n,p);
    sensor[i]=constrain(sensor[i],0,255);
    w=i*sensor[i];
    suma_w+=w;
    sum+=sensor[i];        
  }
  e=suma_w/sum;
  e=map(e,0,15,15,0);
  x=e*500;
  return x;
}

void Blackbar::printValues()
{
  suma_w=0;
  sum=0;
  for(i=15;i>=0;i--)
  {
    sensor[i]=map(readSensor_raw(i),valor_min[i],valor_max[i],n,p);
    sensor[i]=constrain(sensor[i],0,255);  
    w=i*sensor[i];
    suma_w+=w;
    sum+=sensor[i];
    Serial.print(sensor[i]) ;
    Serial.print("\t");     
  }
  e=suma_w/sum;
  e=map(e,0,15,15,0);
  x=e*500;
  Serial.println(x);
}