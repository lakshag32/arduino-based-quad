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
int pitch_prev;
int p_gain = 3;
int p_output;
int i_gain = 1;
int i_output;
int d_gain = 1;
int d_output; 
int pid_output;
int error_i; 
int error;
int previous_error = 0; 
int max_speed = 2310; 
float pitch;
float roll;
float yaw;

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
   off_value = map(channel_3_signal_time,989,1967,55,182);
   if (off_value > 70){
   //if (channel_3_signal_time> 57){
    
     off_value_2 = 4;
     bm3.write(off_value);
     bm4.write(off_value+1);
     bm1.write(off_value+1);
     bm2.write(off_value-31);
     //-32 -1
     mpu.update();
     if((millis()-timer)>10){ // print data every 10ms
            pitch = mpu.getAngleX();
            roll = mpu.getAngleY();
            Serial.print("pitch: ");
            Serial.print(pitch);
            Serial.print('\n');
            //Serial.print("       ");
            //Serial.print("roll: ");
            //Serial.print(roll);
            //Serial.print('\n');
            timer = millis();
      }
      
      if (pitch >2){
          error = pitch - 0; //0 is the angle that we want the drone to be at, if we want the drone to be at a different angle(for example, to move forward), subtract the pitch by that angle
          p_output = error*p_gain+off_value;
          bm4.write(p_output);
          bm2.write(p_output);
          
       }
      if (pitch <-2){
          error = (pitch - 0)*-1; //0 is the angle that we want the drone to be at, if we want the drone to be at a different angle(for example, to move forward), subtract the pitch by that angle
          p_output = error*p_gain+off_value;
          bm3.write(p_output);
          bm1.write(p_output);  
      
        }
      
      if (roll <-2){
          error = (roll - 0)*-1; //0 is the angle that we want the drone to be at, if we want the drone to be at a different angle(for example, to move forward), subtract the pitch by that angle
          p_output = error*p_gain+off_value;
          bm3.write(p_output);
          bm4.write(p_output);  
      
        }
      if (roll > 2){
          error = roll - 0; //0 is the angle that we want the drone to be at, if we want the drone to be at a different angle(for example, to move forward), subtract the pitch by that angle
          p_output = error*p_gain+off_value;
          bm2.write(p_output);
          bm1.write(p_output);    

           
        }
    
   }
}
