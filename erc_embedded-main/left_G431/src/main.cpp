#include <Arduino.h>
#include "HardwareSerial.h"
#include "VescUart.h"
#include "ros.h"
#include "std_msgs/Float64MultiArray.h"

//RIGHT NOW IT'S ON DUTY CYCLE MDOE

#define MOTOR_NUM 2

//UART ports
HardwareSerial LFserial(PC5, PC4);
HardwareSerial LBserial(PB11, PB10);

//VESC objects
VescUart LFmotor;
VescUart LBmotor;

//Standart variables
float left_motors_command_float_array[MOTOR_NUM];

//ROS variables
std_msgs::Float64MultiArray left_motors_feedback_multiarray;
std_msgs::Float64MultiArray left_motors_command_multiarray;

//Function declarations
void leftMotorsCallback(const std_msgs::Float64MultiArray &drive_system_command_multiarray);

//ROS objects
ros::NodeHandle nh;
ros::Subscriber <std_msgs::Float64MultiArray> leftMotorsSubscriber("drive_system/wheel_speed", &leftMotorsCallback);
ros::Publisher leftMotorsPublisher("drive_system_left_motors_feedbacks", &left_motors_feedback_multiarray);

void setup() {
  //ROS inits
  nh.initNode();
  nh.subscribe(leftMotorsSubscriber);
  nh.advertise(leftMotorsPublisher);

  //UART Baud rates
  LFserial.begin(115200);
  LBserial.begin(115200);

  //VESC-UART declarations
  LFmotor.setSerialPort(&LFserial);
  LBmotor.setSerialPort(&LBserial);

  //Memory allocation of the feedback multiarray
  left_motors_feedback_multiarray.data=(float *)malloc(sizeof(float)*MOTOR_NUM);  
  left_motors_feedback_multiarray.data_length=MOTOR_NUM;

  left_motors_command_multiarray.data=(float *)malloc(sizeof(float)*MOTOR_NUM);  
  left_motors_command_multiarray.data_length=MOTOR_NUM;
}

void loop() {
  nh.spinOnce();
  delay(1);

  LFmotor.setDuty(left_motors_command_float_array[0]);
  LBmotor.setDuty(left_motors_command_float_array[1]);
}

void leftMotorsCallback(const std_msgs::Float64MultiArray &drive_system_command_multiarray){
  /*
  left_motors_command_float_array[0]=drive_system_command_multiarray.data[1]-drive_system_command_multiarray.data[0];
  left_motors_command_float_array[1]=drive_system_command_multiarray.data[1]-drive_system_command_multiarray.data[0];
  */
  left_motors_command_float_array[0]=drive_system_command_multiarray.data[0];
  left_motors_command_float_array[1]=drive_system_command_multiarray.data[0];

  for(int motor_counter=0;motor_counter<MOTOR_NUM;motor_counter++){
    if(left_motors_command_float_array[motor_counter]>=1){
      left_motors_command_float_array[motor_counter]=1;
    }
    if(left_motors_command_float_array[motor_counter]<=-1){
      left_motors_command_float_array[motor_counter]=-1;
    }
  }

  if(LFmotor.getVescValues() && LBmotor.getVescValues()){
    left_motors_feedback_multiarray.data[0]=float(LFmotor.data.rpm/70);
    left_motors_feedback_multiarray.data[1]=float(LBmotor.data.rpm/70);
  }

  leftMotorsPublisher.publish(&left_motors_feedback_multiarray);
}
