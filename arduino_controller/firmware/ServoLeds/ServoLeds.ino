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
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>

// LEDS
#define LED_ON    2
#define LED_ALERT 3
int led_bat[4] = {4, 5, 6, 7};
#define LED_SAFETY 8
#define LED_TELEOP 10
//SERVO
#define SERVO_PIN 9

int oldpos = 90;
int pos = 90;
int newpos = 90;
unsigned long previousMillis = 0;
const long interval = 8;//delay for servo 'step'  
ros::NodeHandle  nh;


Servo servo;

void servo_cb( const std_msgs::Int16 & cmd_msg) {
  newpos = cmd_msg.data;



  //servo.write(cmd_msg.data); //set servo angle, should be from 0-180
  //digitalWrite(13, HIGH - digitalRead(13)); //toggle led

}

void led_on_cb( const std_msgs::Bool & cmd_msg) {
  digitalWrite(LED_ON, cmd_msg.data); //set led
}
void led_alert_cb( const std_msgs::Bool & cmd_msg) {
  digitalWrite(LED_ALERT, cmd_msg.data); //set led
}
void led_safety_cb( const std_msgs::Bool & cmd_msg) {
  digitalWrite(LED_SAFETY, cmd_msg.data); //set led
}
void led_teleop_cb( const std_msgs::Bool & cmd_msg) {
  digitalWrite(LED_TELEOP, cmd_msg.data); //set led
}
void bat_per_cb( const std_msgs::UInt8 & cmd_msg) {
  float index = (float)cmd_msg.data / 25.0;
  int bi = round(index); // [0,4]
  for (int i = 0; i<bi; i++){
    digitalWrite(led_bat[i], HIGH);
  }
  for (int i = bi; i<4; i++){
    digitalWrite(led_bat[i], LOW);
  }
}

ros::Subscriber<std_msgs::Int16> sub("servo", servo_cb);
ros::Subscriber<std_msgs::Bool> sub_lo("led/on", led_on_cb);
ros::Subscriber<std_msgs::Bool> sub_le("led/alert", led_alert_cb);
ros::Subscriber<std_msgs::Bool> sub_ls("led/safety", led_safety_cb);
ros::Subscriber<std_msgs::Bool> sub_lt("led/teleop", led_teleop_cb);
ros::Subscriber<std_msgs::UInt8> sub_batt("batt_per", bat_per_cb);

void setup() {
  //set LEDS
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_ON, OUTPUT);
  pinMode(LED_ALERT, OUTPUT);
  pinMode(LED_SAFETY, OUTPUT);
  pinMode(LED_TELEOP, OUTPUT);
  for (int i = 0; i<4; i++){
    pinMode(led_bat[i], OUTPUT);
  }
  Serial.begin(115200);// set baud
  nh.getHardware()->setBaud(115200);

  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub_lo);
  nh.subscribe(sub_le);
  nh.subscribe(sub_ls);
  nh.subscribe(sub_lt);
  nh.subscribe(sub_batt);

  servo.attach(SERVO_PIN); //attach it to pin 9


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
