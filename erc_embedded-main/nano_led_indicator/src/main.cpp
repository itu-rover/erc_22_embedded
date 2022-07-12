#include <Arduino.h>
#include "ros.h"
#include "std_msgs/String.h"

#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 5

std_msgs::String incoming_string;
String my_Str;

void red();
void yellow();
void green();
void turn_off();
void ledCallback(const std_msgs::String &incoming_string);

ros::NodeHandle nh;
ros::Subscriber<std_msgs::String> led_Sub("led_topic", ledCallback);

void setup() {
  nh.initNode();
  nh.subscribe(led_Sub);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  
  nh.spinOnce();
  delay(1);
  
}

void red(void){
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
}

void yellow(void){
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
}

void green(void){
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
}

void turn_off(void){
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
}

void ledCallback(const std_msgs::String &incoming_string){
  my_Str=incoming_string.data;
  if(my_Str=="R"){
    red();
  }
  if(my_Str=="Y"){
    yellow();
  }
  if(my_Str=="G"){
    green();
  }
  if(my_Str=="T"){
    turn_off();
  }
  delay(10);
}
