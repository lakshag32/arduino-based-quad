#include <Servo.h>
int counter = 0; 
int channel_1_pwm_pin = 4;
int channel_2_pwm_pin = 8;
int channel_3_pwn_pin = 7;  
int channel_4_pwm_pin = 2; 
int pwm_pin_3 =3;
int pwm_pin_5 =5;
int pwm_pin_6 =6;
int pwm_pin_9 =9;

int channel_1_signal_time; 
int channel_2_signal_time;
int channel_3_signal_time;
int channel_4_signal_time;

   
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
  
void loop(){    
   
   channel_3_signal_time = pulseIn(channel_3_pwn_pin, HIGH);  
   motor_speed = map(channel_3_signal_time,990,1965,50,90);  
   bm3.write(motor_speed);
   bm4.write(motor_speed);
   bm1.write(motor_speed);
   bm2.write(motor_speed);
  
   
  
     
              
        
        
             
    
}
