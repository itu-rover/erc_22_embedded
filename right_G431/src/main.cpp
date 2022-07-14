#include <Arduino.h>
#include "HardwareSerial.h"
#include "VescUart.h"
#include "ros.h"
#include "std_msgs/Float64MultiArray.h"

#define MOTOR_NUM 2
#define CURRENT_COEFF 6
#define RPM_COEFF 10000

//RIGHT NOW IT'S ON DUTY CYCLE or CURRENT MODE

//UART ports
HardwareSerial RFserial(PC5, PC4);
HardwareSerial RBserial(PB11, PB10);

//VESC objects
VescUart RFmotor;
VescUart RBmotor;

//Standart variables
float right_motors_command_float_array[MOTOR_NUM];
float right_motors_current_command_float_array[MOTOR_NUM];

//ROS variables
std_msgs::Float64MultiArray right_motors_feedback_multiarray;
std_msgs::Float64MultiArray right_motors_command_multiarray;

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

  right_motors_command_multiarray.data=(float *)malloc(sizeof(float)*MOTOR_NUM);  
  right_motors_command_multiarray.data_length=MOTOR_NUM;
}

void loop() {
  nh.spinOnce();
  delay(1);

  RFmotor.setDuty(right_motors_command_float_array[0]);
  RBmotor.setDuty(right_motors_command_float_array[1]);
  
  /*
  RFmotor.setCurrent(right_motors_current_command_float_array[0]);
  RFmotor.setCurrent(right_motors_current_command_float_array[1]);
  */
  /*
  RFmotor.setRPM(right_motors_command_float_array[0]);
  RBmotor.setRPM(right_motors_command_float_array[1]);
  */
}

void rightMotorsCallback(const std_msgs::Float64MultiArray &drive_system_command_multiarray){
  for(int motor_counter=0;motor_counter<MOTOR_NUM;motor_counter++){
    right_motors_command_float_array[motor_counter]=drive_system_command_multiarray.data[1];
    right_motors_current_command_float_array[motor_counter]=drive_system_command_multiarray.data[1]*CURRENT_COEFF;
  }

  if(RFmotor.getVescValues() && RBmotor.getVescValues()){
    right_motors_feedback_multiarray.data[0]=float(RFmotor.data.rpm/70);
    right_motors_feedback_multiarray.data[1]=float(RBmotor.data.rpm/70);
  }

  rightMotorsPublisher.publish(&right_motors_feedback_multiarray);
}
