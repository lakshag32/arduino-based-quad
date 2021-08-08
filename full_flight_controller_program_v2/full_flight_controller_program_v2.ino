#include <Servo.h>


#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
unsigned long timer = 0;



int channel_1_pwm_pin = 4;
int channel_2_pwm_pin = 8;
int channel_3_pwn_pin = 7;  
int channel_4_pwm_pin = 2; 
int pwm_pin_3 =3;
int pwm_pin_5 =5;
int pwm_pin_6 =6;
int pwm_pin_9 =9;
int min_speed = 64;
int off_value = 63;
int off_value_3 = 63;
int medium_speed = 90;
int max_speed = 180; 

int reciever_degree = 0;

int p_gain = 1;
int i_gain = 1 ;
int d_gain = 0;
int p_output = 0;
int i_output = 0;
int d_output = 0;
int pitch_prev = 0;
int pid_output = 0;
int pid_output2 = 0;
int reciever_degree_prev = 0;

int p_gain2 = 1;
int i_gain2 = 1;
int d_gain2 = 0;
int p_output2 = 0;
int i_output2 = 0;
int d_output2 = 0;
int roll_prev = 0;
int reciever_degree_prev2 = 0;
float pitch;
float roll;
float yaw;
unsigned long channel_1_signal_time; 
unsigned long channel_2_signal_time;
unsigned long channel_3_signal_time;
unsigned long channel_4_signal_time;

Servo bm1;
Servo bm2;
Servo bm3;
Servo bm4;



void setup() {
  Serial.begin(9600);
  pinMode(channel_1_pwm_pin,INPUT);
  pinMode(channel_1_pwm_pin,INPUT);
  pinMode(channel_1_pwm_pin,INPUT);
  pinMode(channel_1_pwm_pin,INPUT);
  pinMode(pwm_pin_3, OUTPUT);
  pinMode(pwm_pin_5, OUTPUT);
  pinMode(pwm_pin_6, OUTPUT);
  pinMode(pwm_pin_9, OUTPUT);
  bm1.attach(pwm_pin_3);
  bm2.attach(pwm_pin_5);
  bm3.attach(pwm_pin_6);
  bm4.attach(pwm_pin_9);
  //arm the escs
  bm1.write(off_value_3);
  bm2.write(off_value_3);
  bm3.write(off_value_3);
  bm4.write(off_value_3);
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
  
  
}

void loop(){    
    channel_3_signal_time = pulseIn(channel_3_pwn_pin, HIGH); 
    off_value = map(channel_3_signal_time,992,1968,63,180);

    bm1.write(off_value);
    bm2.write(off_value);
    bm3.write(off_value);
    bm4.write(off_value);
    if((millis()-timer)>10){
      mpu.update();
      pitch = mpu.getAngleX();
      roll = mpu.getAngleY();
      Serial.print("pitch: ");
  
      Serial.print(pitch);
      Serial.print("\n");
      Serial.print("roll: ");
      Serial.print(roll);
      Serial.print("\n");
    }
    p_output = pitch-(0)*p_gain;
    i_output = i_output+(pitch-(0)*i_gain);
    d_output = (pitch-0-pitch_prev+reciever_degree_prev)*d_gain;
    pitch_prev = pitch;
    reciever_degree_prev = 0;
    pid_output = p_output+i_output+d_output;
    if (pitch > 0){
      bm1.write(pid_output);
      bm3.write(pid_output);
    }
    if (pitch < 0){
      bm4.write(pid_output);
      bm2.write(pid_output);
    }
    p_output2 = roll-(0)*p_gain2;
    i_output2 = i_output2+(roll-(0)*i_gain2);
    d_output2 = (roll-0-roll_prev+reciever_degree_prev2)*d_gain2;
    roll_prev = roll;
    reciever_degree_prev2 = 0;
    pid_output2 = p_output2+i_output2+d_output2;
    if (roll > 0){
      bm2.write(pid_output2);
      bm3.write(pid_output2);
    }
    if (roll < 0){
      bm1.write(pid_output2);
      bm4.write(pid_output2);
    }
}
