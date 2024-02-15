#include <ros.h>
#include <geometry_msgs/Quaternion.h>

#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

SoftwareSerial SWSerial1(NOT_A_PIN,11); // RX on no pin (unused), TX on pin 11 (to S1_1). data transmition
SoftwareSerial SWSerial2(NOT_A_PIN, 5); // RX on no pin (unused), TX on pin 5 (to S1_2).

ros::NodeHandle  nh;

void Velocidadmsg( const geometry_msgs::Quaternion& omni_vel){
  int wheel_1, wheel_2, wheel_3, wheel_4;
  wheel_1 = map(omni_vel.x,-10,10,128,255);
  wheel_2 = map(omni_vel.w,-10,10,1  ,127);
  wheel_3 = map(omni_vel.y,-10,10,128,255);
  wheel_4 = map(omni_vel.z,-10,10,1  ,127);

  //SET Motor 1 and Motor 3
  digitalWrite(1,HIGH); //S2_1
  digitalWrite(2,HIGH); //S2_2
  
  SWSerial1.write(wheel_1); //Motor 1
  SWSerial2.write(wheel_3); //Motor 3

  //SET Motor 2 and Motor 4
  SWSerial1.write(wheel_2); //Motor 2
  SWSerial2.write(wheel_4); //Motor 4

  digitalWrite(1,LOW); //S2_1
  digitalWrite(2,LOW); //S2_2
}

ros::Subscriber<geometry_msgs::Quaternion> sub("omni_vel", &Velocidadmsg);

void setup()
{
  SWSerial1.begin(38400);
  SWSerial2.begin(38400);
  SabertoothTXPinSerial.begin(38400);
  pinMode(1,OUTPUT); //S2_1: when it's high ejecute the information and when it's low don't do anything.
  pinMode(2,OUTPUT); //S2_2: when it's high ejecute the information and when it's low don't do anything.
  
  nh.initNode();
  nh.subscribe(sub);
 
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
