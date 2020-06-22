/*
   rosserial Servo Control Example

   This sketch demonstrates the control of hobby R/C servos
   using ROS and the arduiono

   For the full tutorial write up, visit
   www.ros.org/wiki/rosserial_arduino_demos

   For more information on the Arduino Servo Library
   Checkout :
   http://www.arduino.cc/en/Reference/Servo
*/

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int16.h>

int oldpos = 90;
int pos = 90;
int newpos = 90;
unsigned long previousMillis = 0;
const long interval = 8;   
ros::NodeHandle  nh;

Servo servo;

void servo_cb( const std_msgs::Int16 & cmd_msg) {
  newpos = cmd_msg.data;



  //servo.write(cmd_msg.data); //set servo angle, should be from 0-180
  digitalWrite(13, HIGH - digitalRead(13)); //toggle led

}


ros::Subscriber<std_msgs::Int16> sub("servo", servo_cb);

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(115200);// set baud
  nh.getHardware()->setBaud(115200);

  nh.initNode();
  nh.subscribe(sub);

  servo.attach(9); //attach it to pin 9


  servo.write(90); //default position
}

void loop() {
  nh.spinOnce();
   unsigned long currentMillis = millis();
     if (currentMillis - previousMillis >= interval) {
      
         previousMillis = currentMillis;
  if (pos < newpos) {
    pos += 1;
    if (pos > 180) {
      pos == 180;
    }
    servo.write(pos);
  }

  if (pos > newpos) {
    pos -= 1;
    if (pos < 0) {
      pos == 0;
    }
    servo.write(pos);
  }

     }
     //delay(1);
}
