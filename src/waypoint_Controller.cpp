#include <cmath>
#include <math.h>
#include "ros/ros.h"
#include <ros/console.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include <geometry_msgs/Pose2D.h>
#include "geometry_msgs/Vector3.h"
#include <std_msgs/Float64.h>
#include <vector>
#include <cstdlib>
#include <iostream>
  
  class Pose{
  public:

  // 2D robot pose in the plane
    static double _X;
    static double _Y;
    static double _Theta;
    
  Pose();
  ~Pose();

  }; // class

  // Initialize pose values (in case we try to calculate before calling CB)
  double Pose::_X = 0.0;
  double Pose::_Y = 0.0;
  double Pose::_Theta = 0.0;
  
  
  void poseCB(const geometry_msgs::Pose2DConstPtr& msg)
  {
  Pose::_X = msg->x;
  Pose::_Y = msg->y;
  Pose::_Theta = msg->theta;
  }
  
int main(int argc, char **argv)
{

  double pi = 3.14159265359;

  //***FILL IN GIVEN WAYPOINT VALUES**********
  
  double x_wp, y_wp;
  x_wp = 1.0;
  y_wp = 1.0; //actually z in openMVG coordinates
  
  //********************************************
  
  ros::init(argc, argv, "waypoint_Controller");

  ros::NodeHandle n;

  geometry_msgs::Twist cmd; //create message objects
  geometry_msgs::Pose2D pose_calc;
  
  //Initialize commands
  cmd.linear.x = 0.0;
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;
  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;
  cmd.angular.z = 0.0;
  
  //Initialize control system parameters
  double k1, k2, k3;
  k1 = 0.05; //adjust speed to approach the target
  k2 = 0.05; //adjust turning rate to face the target
  k3 = 0.05; //adjust
  
  //Velocity limits to apply before pub
  double fwd_max, turn_max;
  fwd_max = 0.8;
  turn_max = 0.25;
  
  ros::Rate loop_rate(10);
  
  ros::Subscriber sub_pose = n.subscribe<geometry_msgs::Pose2D>("dr_pose_update", 1, poseCB);
  
  ros::Publisher pub_cmd = n.advertise<geometry_msgs::Twist>("atlas/cmd_vel", 1000);
  
while (ros::ok())
  {
  
  //***Control Law********************************
  
  double rho, alpha, beta, x_err, y_err, theta_err;
  
  x_err = x_wp - Pose::_X;
  y_err = y_wp - Pose::_Y;
    
  rho = sqrt( pow(x_err,2.0) + pow(y_err,2.0) );
  
  alpha = -1.0*Pose::_Theta + atan2(y_err,x_err);
  
  // Make corrections so that alpha is between -pi, pi
 
  if (alpha > 2.0*pi){
  alpha = fmod(alpha,2*pi); //modulus
  }
  else if (alpha > pi){
  alpha = -1.0*pi + fmod(alpha,pi);
  }
  else if (alpha < -2.0*pi){
  alpha = fmod(alpha,-2.0*pi);
  }
  else if (alpha < -1.0*pi){
  alpha = pi - fmod(alpha,-1.0*pi);
  }
  
  beta = -1.0*Pose::_Theta - alpha;
    
  //Update commands
  cmd.linear.y = k1*rho; //fwd velocity command
  cmd.angular.z = k2*alpha + k3*beta; //angular velocity command
  
  if (cmd.linear.y > fwd_max)
  cmd.linear.y = 1.0;
  
  if (cmd.linear.y > turn_max)
  cmd.linear.y = 1.0;
  
  pub_cmd.publish(cmd);

  ros::spinOnce();

  loop_rate.sleep();
  
  } //while loop
  
} //main
  
  
  
