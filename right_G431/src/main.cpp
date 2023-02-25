#include <Arduino.h>
#include "HardwareSerial.h"
#include "VescUart.h"
#include "ros.h"
#include "std_msgs/Float64MultiArray.h"

#define MOTOR_NUM 2
#define RPM_COEFF 10000

//UART ports
HardwareSerial RFserial(PC5, PC4);
HardwareSerial RBserial(PB11, PB10);

//VESC objects
VescUart RFmotor;
VescUart RBmotor;

//Standart variables
float right_front_motor_speed;
float right_back_motor_speed;
unsigned long callback_start_time;
//angel variable
float angle;

//ROS variables
std_msgs::Float64MultiArray right_motors_feedback_multiarray;

//Function declarations
void rightMotorsCallback(const std_msgs::Float64MultiArray &drive_system_command_multiarray);

//ROS objects
ros::NodeHandle nh;
ros::Subscriber <std_msgs::Float64MultiArray> rightMotorsSubscriber("drive_system/wheel_speed", &rightMotorsCallback);
ros::Publisher rightMotorsPublisher("drive_system_right_motors_feedbacks", &right_motors_feedback_multiarray);

void setup() {
  //ROS inits
  nh.initNode();
  nh.subscribe(rightMotorsSubscriber);
  nh.advertise(rightMotorsPublisher);

  //UART Baud rates
  RFserial.begin(115200);
  RBserial.begin(115200);

  //VESC-UART declarations
  RFmotor.setSerialPort(&RFserial);
  RBmotor.setSerialPort(&RBserial);

  //Memory allocation of the feedback multiarray
  right_motors_feedback_multiarray.data=(float *)malloc(sizeof(float)*MOTOR_NUM);  
  right_motors_feedback_multiarray.data_length=MOTOR_NUM;

  callback_start_time=millis();
}

void loop() {
  nh.spinOnce();
  delay(1);

  unsigned long loop_start_time=millis();
  
  if(!nh.connected() || (loop_start_time-callback_start_time)>500){
    RFmotor.setRPM(0);
    RBmotor.setRPM(0);
  }


  //Brake according to the ramp angle
  else if((nh.connected()) && fabs(right_front_motor_speed) <= 2 && fabs(right_back_motor_speed) <= 2 && angle>5)
  {
    RFmotor.setHandBrakeCurrent(6); 
    RBmotor.setHandBrakeCurrent(6);
  }

  else{
    RFmotor.setRPM(right_front_motor_speed);
    RBmotor.setRPM(right_back_motor_speed);
  }
}

void rightMotorsCallback(const std_msgs::Float64MultiArray &drive_system_command_multiarray){
  callback_start_time=millis();
  angle = drive_system_command_multiarray.data[4];
  right_front_motor_speed = drive_system_command_multiarray.data[2]*RPM_COEFF;
  right_back_motor_speed = drive_system_command_multiarray.data[3]*RPM_COEFF;

  if(RFmotor.getVescValues() && RBmotor.getVescValues()){
    right_motors_feedback_multiarray.data[0]=float(RFmotor.data.rpm/70);
    right_motors_feedback_multiarray.data[1]=float(RBmotor.data.rpm/70);
  }

  rightMotorsPublisher.publish(&right_motors_feedback_multiarray);
}
