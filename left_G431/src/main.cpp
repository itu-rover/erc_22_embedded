#include <Arduino.h>
#include "HardwareSerial.h"
#include "VescUart.h"
#include "ros.h"
#include "std_msgs/Float64MultiArray.h"

#define MOTOR_NUM 2
#define RPM_COEFF 10000

//UART ports
HardwareSerial LFserial(PC5, PC4);
HardwareSerial LBserial(PB11, PB10);

//VESC objects
VescUart LFmotor;
VescUart LBmotor;

//Standart variables
float left_front_motor_speed;
float left_back_motor_speed;
unsigned long callback_start_time;
//angel variable
float angle;

//ROS variables
std_msgs::Float64MultiArray left_motors_feedback_multiarray;

//Function declarations
void leftMotorsCallback(const std_msgs::Float64MultiArray &drive_system_command_multiarray);

//ROS objects
ros::NodeHandle nh;
ros::Subscriber <std_msgs::Float64MultiArray> leftMotorsSubscriber("drive_system/wheel_speed", &leftMotorsCallback);
ros::Publisher leftMotorsPublisher("drive_system_left_motors_feedbacks", &left_motors_feedback_multiarray);

void setup() 
{
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

  callback_start_time=millis();
}

void loop() 
{
  nh.spinOnce();
  delay(1);

  unsigned long loop_start_time=millis();
  
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
}

void leftMotorsCallback(const std_msgs::Float64MultiArray &drive_system_command_multiarray)
{
  callback_start_time=millis();
  angle = drive_system_command_multiarray.data[4];
  left_front_motor_speed = drive_system_command_multiarray.data[0]*RPM_COEFF;
  left_back_motor_speed = drive_system_command_multiarray.data[1]*RPM_COEFF;

  if(LFmotor.getVescValues() && LBmotor.getVescValues())
  {
    left_motors_feedback_multiarray.data[0]=float(LFmotor.data.rpm/70);
    left_motors_feedback_multiarray.data[1]=float(LBmotor.data.rpm/70);
  }

  leftMotorsPublisher.publish(&left_motors_feedback_multiarray);
}
