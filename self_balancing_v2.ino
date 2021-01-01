#include <Wire.h>
#include "PWM.hpp"

PWM ROLL(2);
PWM PITCH(3);

double Kp=36.6, Ki=0.36,Kd=780.05;
//double Kp=33.0, Ki=-0.29,Kd=780.05;
//double Kp=41.0, Ki=-0.135,Kd=890.05;
double p,i,d,pid,ut;
const int maxsp=255;
const int minsp =-255;
float sp=0.670,pid_sp;
int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];
unsigned long elapsedTime[2], time1[2], timePrev[2];
double error,pre_error;
float rad_to_deg = 180/PI;
const byte left1_motor = 5;
const byte left2_motor = 6;
const byte right1_motor = 9;
const byte right2_motor = 10;
const byte enable_remote = 4;

void setup() {
  
  Wire.begin();                       //begin the wire comunication
  Wire.beginTransmission(0x68);       //call address mpu6050
  Wire.write(0x6B);
  Wire.write(0);                      // write 0 to setup 16-bits data
  Wire.endTransmission(true);
  Serial.begin(9600);
  ROLL.begin(true);
  PITCH.begin(true);
  Serial.setTimeout(100);
  pinMode(left1_motor,OUTPUT);
  pinMode(left2_motor,OUTPUT);
  pinMode(right1_motor,OUTPUT);
  pinMode(right2_motor,OUTPUT);
//  pinMode(enable_remote,INPUT);
//  pinMode(7,OUTPUT);
//  digitalWrite(7,1);
}

void loop() {
 
   Wire.beginTransmission(0x68);              // address of mpu6050
   Wire.write(0x3B);                          // write an address of acceleration
   Wire.endTransmission(false);               // stop transmitting
   Wire.requestFrom(0x68,6,true);             // request 6 values from mpu6050

   Acc_rawX=Wire.read()<<8|Wire.read();       // get acc_rawX value (high and low significant bit)
   Acc_rawY=Wire.read()<<8|Wire.read();       // get acc_rawY value (high and low significant bit)
   Acc_rawZ=Wire.read()<<8|Wire.read();       // get acc_rawZ value (high and low significant bit)
  
   Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;     // convert acc_rawX value to acceleration_anlge
   Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;  // convert acc_rawY value to acceleration_anlge
   
   Wire.beginTransmission(0x68);              // address of mpu6050
   Wire.write(0x43);                          // write an address of Gyrow 
   Wire.endTransmission(false);               // stop transmitting
   Wire.requestFrom(0x68,4,true);             // request 4 values from mpu6050
   
   Gyr_rawX=Wire.read()<<8|Wire.read();       // get Gyr_rawX value (high and low significant bit)
   Gyr_rawY=Wire.read()<<8|Wire.read();       // get Gyr_rawY value (high and low significant bit)

   Gyro_angle[0] = Gyr_rawX/131.0;            // convert Gyr_rawX value to Gyr_anlge
   Gyro_angle[1] = Gyr_rawY/131.0;            // convert Gyr_rawX value to Gyr_anlge

   // using complementary filter to get the both final angles X and Y.
   time1[0] = millis();                          // start counting time
   elapsedTime[0] = (time1[0] - timePrev[0])/1000; 
   Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime[0]) + 0.02*Acceleration_angle[0];
   Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime[0]) + 0.02*Acceleration_angle[1];
   

   
   elapsedTime[1] = (time1[0] - timePrev[1]);
   error = sp-Total_angle[0];
   p = Kp * error;
   i = Ki*( i + error*elapsedTime[1]);
   d = Kd *(error - pre_error)/elapsedTime[1];
   ut =  p +i +  d;
   //ut = constrain(ut,minsp,maxsp);
   if (ut>255)ut=255;
   if (ut<-255)ut=-255;
   pre_error = error;
   timePrev[1] = time1[0];
   timePrev[0] = time1[0];
   
   Serial.print(Total_angle[0]);
   Serial.print(" ");
   Serial.println(ut);
   if ( -20 < Total_angle[0] && Total_angle[0] < 20 ){
      if ( ut < 0 ){
        foward();
      }
      if( ut > 0 ){
        backward();
      }
   }
   else {
    Stop();
   } 
}

void foward(){
  analogWrite(left1_motor, abs(ut));
  analogWrite(left2_motor, 0);
  analogWrite(right1_motor, abs(ut));
  analogWrite(right2_motor,0);
}
void backward() {
  analogWrite(left1_motor,0);
  analogWrite(left2_motor, ut);
  analogWrite(right1_motor,0);
  analogWrite(right2_motor, ut);
}
void Stop(){
  analogWrite(6,0);
  analogWrite(9,0);
  analogWrite(10,0);
  analogWrite(11,0);

}
