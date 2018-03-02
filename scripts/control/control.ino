#include <Encoder.h>
#include <ros.h>
#include <std_msgs/Float64.h>
#include "DualMC33926MotorShield.h"

DualMC33926MotorShield md;

Encoder myEncl1(3, 5);
Encoder myEncr1(13, 11);

const float pi = 3.14159;
float r = 0.075;

ros::NodeHandle nh;

// Real speed variables
// Set speed variables

std_msgs::Float64 motor_left_speed1;
std_msgs::Float64 motor_left_speed_set1;

std_msgs::Float64 motor_right_speed1;
std_msgs::Float64 motor_right_speed_set1;

std_msgs::Float64 motor_setpoint_speed;

// Callbacks for motor speed
void set_left_speed1( const std_msgs::Float64& motor_left_speed_set1){
  // Left motor1
  md.setM2Speed(map(motor_left_speed_set1.data, -1, 1, -400,400));
}
void set_right_speed1( const std_msgs::Float64& motor_right_speed_set1){
  // Right motor1
  md.setM1Speed(map(motor_right_speed_set1.data, -1, 1, -400,400));
}


// Real velocity publishers
// Velocity subscribers

ros::Publisher motor_left1("/left_wheel1/state", &motor_left_speed1);
ros::Subscriber<std_msgs::Float64> motor_left_controller1("/left_wheel1/control_effort", &set_left_speed1);

ros::Publisher motor_right1("/right_wheel1/state", &motor_right_speed1);
ros::Subscriber<std_msgs::Float64> motor_right_controller1("/right_wheel1/control_effort", &set_right_speed1);

ros::Publisher setpoint_left ("/left_wheel1/setpoint", &motor_setpoint_speed);
ros::Publisher setpoint_right ("/right_wheel1/setpoint", &motor_setpoint_speed);


long oldPositionr1  = -999, oldPositionl1  = -999;
float radsr1 = 0.0, radsl1 = 0.0;
long newPositionr1, newPositionl1;

void setup()
{
  md.init();
  // Right motor1
  md.setM1Speed(0);
  
  // Left motor2
  md.setM2Speed(0);
  
  nh.initNode();
  
  nh.advertise(motor_left1);
  nh.subscribe(motor_left_controller1);

  nh.advertise(motor_right1);
  nh.subscribe(motor_right_controller1);

  nh.advertise(setpoint_right);
  nh.advertise(setpoint_left);
}

void loop()
{
  // Right motor speed1
  newPositionr1 = myEncr1.read();
  if (newPositionr1 != oldPositionr1) {
    radsr1 = abs(oldPositionr1-newPositionr1)*(pi/100.0);
    radsr1 = radsr1*r;
    oldPositionr1 = newPositionr1;
  }else{
    radsr1=0;
  }
  motor_right_speed1.data = radsr1;
  
  // LEft motor speed1
  newPositionl1 = myEncl1.read();
  if (newPositionl1 != oldPositionl1) {
    radsl1 = abs(oldPositionl1-newPositionl1)*(pi/100.0);
    radsl1 = radsl1*r;
    oldPositionl1 = newPositionl1;
  }else{
    radsl1=0;
  }
  motor_left_speed1.data = radsl1;

  motor_setpoint_speed.data=0.2;
  
  motor_left1.publish(&motor_left_speed1);
  motor_right1.publish(&motor_right_speed1);
  setpoint_left.publish (&motor_setpoint_speed);
  setpoint_right.publish (&motor_setpoint_speed);
  
  nh.spinOnce();
  delay(100);
}
