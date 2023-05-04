#include <Arduino.h>
#include "ros.h"
#include "std_msgs/Int16MultiArray.h"
#include "HardwareSerial.h"
//#include "VescUart.h"
//#include "std_msgs/Float64MultiArray.h"

#define IN1 3
#define IN2 5
#define IN3 6

//#define MOTOR_NUM 2
//#define RPM_COEFF 10000

//UART ports
//HardwareSerial LFserial(PC11, PC10);
//HardwareSerial LBserial(PD2, PC12);

//VESC objects
//VescUart LFmotor;
//VescUart LBmotor;

//Standart variables
/*
float left_front_motor_speed;
float left_back_motor_speed;
unsigned long callback_start_time;
float LFwatt = 0;
float LBwatt = 0;
float LFoldAmp = 0;
float LBoldAmp = 0;
uint32_t time;
//angel variable
float angle;
*/
//ROS variables
//std_msgs::Float64MultiArray left_motors_feedback_multiarray;

std_msgs::Int16MultiArray incoming_array;

ros::NodeHandle nh;

void ledCallback(const std_msgs::Int16MultiArray &incoming_array);

ros::Subscriber<std_msgs::Int16MultiArray> led_Sub("led_topic", ledCallback);

//Function declarations
//void leftMotorsCallback(const std_msgs::Float64MultiArray &drive_system_command_multiarray);

//ROS objects
//ros::Subscriber <std_msgs::Float64MultiArray> leftMotorsSubscriber("drive_system/wheel_speed", &leftMotorsCallback);
//ros::Publisher leftMotorsPublisher("drive_system_left_motors_feedbacks", &left_motors_feedback_multiarray);

void setup() {
  nh.initNode();
  nh.subscribe(led_Sub);


  //nh.subscribe(leftMotorsSubscriber);
  //nh.advertise(leftMotorsPublisher);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);

  //UART Baud rates
  //LFserial.begin(115200);
  //LBserial.begin(115200);

  //VESC-UART declarations
  //LFmotor.setSerialPort(&LFserial);
  //LBmotor.setSerialPort(&LBserial);

  //Memory allocation of the feedback multiarray
  //left_motors_feedback_multiarray.data = (float *)malloc(sizeof(float)*MOTOR_NUM*4);  
  //left_motors_feedback_multiarray.data_length = MOTOR_NUM*2;

  //callback_start_time=millis();
}

void loop() {
  
  

  nh.spinOnce();
  delay(1);

  //unsigned long loop_start_time=millis();
  /*
  if(!nh.connected() || (loop_start_time-callback_start_time)>500)
  {
    LFmotor.setRPM(0);
    LBmotor.setRPM(0);
  }
  
  //Brake according to the ramp angle
  else if((nh.connected()) && fabs(left_front_motor_speed) <= 2 && fabs(left_back_motor_speed) <= 2 && angle>5)
  {
    LFmotor.setHandBrakeCurrent(6); // give 3 amps for braking for 45 degree
    LBmotor.setHandBrakeCurrent(6);
  }
  

  else{
    LFmotor.setRPM(left_front_motor_speed);
    LBmotor.setRPM(left_back_motor_speed);
  }
  delay(1);
  */
}


void ledCallback(const std_msgs::Int16MultiArray &incoming_array){
  analogWrite(IN1, incoming_array.data[1]);
  analogWrite(IN2, incoming_array.data[2]);
  analogWrite(IN3, incoming_array.data[0]);

  delay(10);
}
/*
void leftMotorsCallback(const std_msgs::Float64MultiArray &drive_system_command_multiarray)
{
  callback_start_time=millis();
  angle = drive_system_command_multiarray.data[4];
  left_front_motor_speed = drive_system_command_multiarray.data[0]*RPM_COEFF;
  left_back_motor_speed = drive_system_command_multiarray.data[1]*RPM_COEFF;

  if(LFmotor.getVescValues() && LBmotor.getVescValues())
  {
    left_motors_feedback_multiarray.data[0] = float(LFmotor.data.rpm/70);
    left_motors_feedback_multiarray.data[1] = float(LBmotor.data.rpm/70);
    left_motors_feedback_multiarray.data[2] = float(LFmotor.data.ampHours);
    left_motors_feedback_multiarray.data[3] = float(LBmotor.data.ampHours);
    
    LFwatt += float(LFmotor.data.ampHours - LFoldAmp) * LFmotor.data.inpVoltage;
    LBwatt += float(LBmotor.data.ampHours - LBoldAmp) * LBmotor.data.inpVoltage;
    left_motors_feedback_multiarray.data[4] = LFwatt;
    left_motors_feedback_multiarray.data[5] = LBwatt;
    LFoldAmp = LFmotor.data.ampHours;
    LBoldAmp = LBmotor.data.ampHours;
    left_motors_feedback_multiarray.data[6] = float(LFmotor.data.inpVoltage);
    left_motors_feedback_multiarray.data[7] = float(LBmotor.data.inpVoltage);
    //
  }

  leftMotorsPublisher.publish(&left_motors_feedback_multiarray);
  
}
*/