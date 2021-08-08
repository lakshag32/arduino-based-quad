#include <Servo.h>
#include "Wire.h"
#include <MPU6050_light.h>
   
MPU6050 mpu(Wire);
int motor_speed = 180;
int increased_value; 
int channel_1_pwm_pin = 4;
int channel_2_pwm_pin = 8;
int channel_3_pwn_pin = 7;  
int channel_4_pwm_pin = 2; 
int pwm_pin_3 =3;
int pwm_pin_5 =5;
int pwm_pin_6 =6;
int pwm_pin_9 =9;
int min_speed = 1275;
int off_value = 63;
int off_value_3 = 57;
int off_value_2 = 0; 
int medium_speed = 1793;
int max_speed = 2310; 
float pitch;
float roll;
float yaw;
float error;
   
unsigned long channel_1_signal_time; 
unsigned long channel_2_signal_time;
unsigned long channel_3_signal_time;
unsigned long channel_4_signal_time;
unsigned long timer = 0;
   
bool condition = true;
   
Servo bm1;
Servo bm2;
Servo bm3;
Servo bm4;
   
void setup(){
  Serial.begin(9600);
  bm1.attach(pwm_pin_3);
  bm2.attach(pwm_pin_5);
  bm3.attach(pwm_pin_6);
  bm4.attach(pwm_pin_9);
  //arm the escs
  bm1.write(4);
  bm2.write(4);
  bm3.write(4);
  bm4.write(4);
  delay(2000);
  
  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
  delay(1000);
  
} 
 
 
void loop(){    
   channel_3_signal_time = pulseIn(channel_3_pwn_pin, HIGH);  
   off_value = map(channel_3_signal_time,989,1967,55,181);
   //increased_value = off_value+20; 
   off_value_2 = 4;
   //off_value_2 = off_value-5;  
   bm3.write(off_value+1);
   bm4.write(off_value+1);
   bm1.write(off_value-31);
   bm2.write(off_value+1);
   
    mpu.update();
    if((millis()-timer)>10){ // print data every 10ms
          pitch = mpu.getAngleX();
          roll = mpu.getAngleY();
          Serial.print("pitch: ");
          Serial.print(pitch);
          Serial.print("        ");
          Serial.print("roll: ");
          Serial.print(roll);
          Serial.print('\n');
          timer = millis(); 
    }
    while(roll<-2){
        bm2.write(off_value_2);
        bm3.write(off_value_2); 
        bm1.write(max_speed); 
        bm4.write(max_speed); 
        mpu.update();
        if((millis()-timer)>10){ // print data every 10ms
            pitch = mpu.getAngleX();
            roll = mpu.getAngleY();
            Serial.print("pitch: ");
            Serial.print(pitch);
            Serial.print("        ");
            Serial.print("roll: ");
            Serial.print(roll);
            Serial.print('\n');
            timer = millis(); 
           }
    
    
      }
     
     while(roll>2){
        bm1.write(off_value_2-32);
        bm4.write(off_value_2); 
        bm3.write(max_speed); 
        bm2.write(max_speed);
        mpu.update();
        if((millis()-timer)>10){ // print data every 10ms
            pitch = mpu.getAngleX();
            roll = mpu.getAngleY();
            Serial.print("pitch: ");
            Serial.print(pitch);
            Serial.print("        ");
            Serial.print("roll: ");
            Serial.print(roll);
            Serial.print('\n');
            timer = millis(); 
           }
    
    
      }
     
     while(pitch<-2){
        bm1.write(off_value_2-32);
        bm3.write(off_value_2); 
        bm4.write(max_speed);
        bm2.write(max_speed); 
        mpu.update();
        if((millis()-timer)>10){ // print data every 10ms
            pitch = mpu.getAngleX();
            roll = mpu.getAngleY();
            Serial.print("pitch: ");
            Serial.print(pitch);
            Serial.print("        ");
            Serial.print("roll: ");
            Serial.print(roll);
            Serial.print('\n');
            timer = millis(); 
           }
    
    
      }   
    
     while(pitch>2){
        bm2.write(off_value_2);
        bm4.write(off_value_2); 
        bm1.write(max_speed); 
        bm3.write(max_speed);
        mpu.update();
        if((millis()-timer)>10){ // print data every 10ms
            pitch = mpu.getAngleX();
            roll = mpu.getAngleY();
            Serial.print("pitch: ");
            Serial.print(pitch);
            Serial.print("        ");
            Serial.print("roll: ");
            Serial.print(roll);
            Serial.print('\n');
            timer = millis(); 
           }
    
    
      }
    
   
} 
