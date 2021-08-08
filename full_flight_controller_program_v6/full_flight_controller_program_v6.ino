#include <Servo.h>
#include "Wire.h"
#include <MPU6050_light.h>
   
MPU6050 mpu(Wire);
int motor_speed = 180;
int counter = 0; 
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

int p_gain1 = 3;
int p_output1;
int i_gain1 = 1;
int i_output1;
int d_gain1 = 1;
int d_output1; 
int pid_output1;
int error_i1; 
int error1;
int previous_error1 = 0; 

int p_gain2 = 3;
int p_output2;
int i_gain2 = 1;
int i_output2;
int d_gain2 = 1;
int d_output2; 
int pid_output2;
int error_i2; 
int error2;
int previous_error2 = 0; 

int p_gain3 = 3;
int p_output3;
int i_gain3 = 1;
int i_output3;
int d_gain3 = 1;
int d_output3; 
int pid_output3;
int error_i3; 
int error3;
int previous_error3 = 0; 

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
            counter += 1; 
          }
      if (counter > 60){
        
            //PID code for when drone pitches in positive direction 
            error = pitch - 0; //0 is the angle that we want the drone to be at, if we want the drone to be at a different angle(for example, to move forward), subtract the pitch by that angle
            error_i += error/2; 
            if (error_i >= 180){
              error_i = 180;  
            }
            if (error  <2 and error >-2){ //set error_i(accumulated error that is used for the i_output) equal to 0 when the drone is pretty much level - some leeway is given to prevent error_i from stacking up sooner and causing motors to spin too fast  
              error_i = 0;
            }
            p_output = error*p_gain+off_value;
            i_output = error_i*i_gain+off_value;
            d_output = (error-previous_error) * d_gain; 
            pid_output = (p_output+i_output+d_output)/2;
            bm4.write(pid_output);
            bm2.write(pid_output);
            previous_error = error; 


           
            //PID code for when drone pitches in negative direction 
            error1 = (pitch - 0)*-1; //0 is the angle that we want the drone to be at, if we want the drone to be at a different angle(for example, to move forward), subtract the pitch by that angle
            error_i1 += error1/2; //divide error by 2 so that error_i won't be too big when error(degrees drone is tilted) is a lot and just won't be as big in general - this helps in keeping motors from reaching max speed when error is medium
            if (error_i1 >= 180){
              error_i1 = 180;  
              }
            if (error1 == 0){ //set error_i(accumulated error that is used for the i_output) equal to 0 when the drone is pretty much level - some leeway is given to prevent error_i from stacking up sooner and causing motors to spin too fast  
              error_i1 = 0;
            }
            p_output1 = error1*p_gain1+off_value;
            i_output1 = error_i1*i_gain1+off_value;
            d_output1 = (error1-previous_error1) * d_gain1; 
            pid_output1 = (p_output1+i_output1+d_output1)/2; //divide by 2 so that motors just don't spin at max speed so soon when drone pitches backward 
            bm3.write(pid_output1);
            bm1.write(pid_output1);  
            previous_error1 = error1; 


            
            //PID code for when drone rolls in negative direction 
            error2 = (roll - 0)*-1; //0 is the angle that we want the drone to be at, if we want the drone to be at a different angle(for example, to move forward), subtract the pitch by that angle
            error_i2 += error2; 
            if (error_i2 >= 180){
              error_i2 = 180;  
            }
            if (error2  <2 and error2 >-2){ //set error_i(accumulated error that is used for the i_output) equal to 0 when the drone is pretty much level - some leeway is given to prevent error_i from stacking up sooner and causing motors to spin too fast  
              error_i2 = 0;
            }
            p_output2 = error2*p_gain2*-1+off_value;
            i_output2 = error_i2*i_gain2+off_value;
            d_output2 = (error2-previous_error2) * d_gain2; 
            pid_output2 = (p_output2+i_output2+d_output2)/2;
            bm3.write(pid_output2);
            bm4.write(pid_output2);  
            previous_error2 = error2; 


        
            //PID code for when drone rolls in positive direction 
            error3 = roll - 0; //0 is the angle that we want the drone to be at, if we want the drone to be at a different angle(for example, to move forward), subtract the pitch by that angle
            error_i3 += error3; 
            if (error_i3 >= 180){
              error_i3 = 180;  
            }
            if (error3  <2 and error3 >-2){ //set error_i(accumulated error that is used for the i_output) equal to 0 when the drone is pretty much level - some leeway is given to prevent error_i from stacking up sooner and causing motors to spin too fast  
              error_i3 = 0;
            }
            p_output3 = error3*p_gain3+off_value;
            i_output3 = error_i3*i_gain3+off_value;
            d_output3 = (error3-previous_error3) * d_gain3; 
            pid_output3 = (p_output3+i_output3+d_output3)/2;
            bm2.write(pid_output3);
            bm1.write(pid_output3);    
            previous_error3 = error3; 
      }
   
}
