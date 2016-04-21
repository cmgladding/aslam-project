#include <cmath>
#include "ros/ros.h"
#include <ros/console.h>
//#include "gazebo_msgs/ModelStates.h"
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
    static double _X_sfm;
    static double _Y_sfm;
    static double _Theta_sfm;
    
    static double _X_dr;
    static double _Y_dr;
    static double _Theta_dr;

  Pose();
  ~Pose();

}; // class

  // Initialize pose values (in case we try to calculate before calling CB)
  double Pose::_X_sfm = 0.0;
  double Pose::_Y_sfm = 0.0;
  double Pose::_Theta_sfm = 0.0;
  
  double Pose::_X_dr = 0.0;
  double Pose::_Y_dr = 0.0;
  double Pose::_Theta_dr = 0.0;
  
  ros::Time last_pose_time;

/*
struct feature_point { // feature point detected by the system
  double x;
  double y;
  double z;

};
*/

// Read data from SfM and update public data values
void poseCB(const geometry_msgs::QuaternionConstPtr& msg)
{
  Pose::_X_sfm = msg->z; //convert default openMVG starting frame
  Pose::_Y_sfm = -1.0*msg->x; //convert default openMVG starting frame
  Pose::_Theta_sfm = msg->y; //actually passing in angle in "y"
  
  //ONLY reset dead reckoning if this is a new pose, not a revised estimate (if #views has increased)
  if (msg->w){  
  Pose::_X_dr = 0.0;
  Pose::_Y_dr = 0.0;
  Pose::_Theta_dr = 0.0;
  }
  
  last_pose_time = ros::Time::now();
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "initController");

  ros::NodeHandle n;

  // Subscribe to the pose estimates produced from SfM
  ros::Subscriber sub_pose = n.subscribe<geometry_msgs::Quaternion>("sfm_pose_update", 1, poseCB);

  // Create publishers for commands, estimates
  ros::Publisher pub_cmd = n.advertise<geometry_msgs::Twist>("atlas/cmd_vel", 1000);
  ros::Publisher pub_dr_calc = n.advertise<geometry_msgs::Pose2D>("dr_pose_update", 1000);

  geometry_msgs::Twist cmd; //create message objects
  geometry_msgs::Pose2D pose_calc;
  
  ros::Rate loop_rate(10);
  

  double x_calc, y_calc, theta_calc; //output variables
  
/*
  // Generate feature points
  std::vector<feature_point> features;
  int num_points = 50;
  for (int length = 0; length < num_points; length+=4) {

  features.push_back(feature_point());
  //generate synthetic wall 1 points
  features[length].x = 10.0*length/num_points - 2.0;
  features[length].y = 1.0;
  features[length].z = 0.0;

  } //for loop - wall generation
*/

// Use publish() function to send message objects
  cmd.linear.x = 0.0; // 0.25 gives a pretty slow motion //initialize forward velocity
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;

  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;
  cmd.angular.z = 0.0; //initialize turning velocity (assume positive z axis points up)
  
  bool start_flag = 0;
  double elapsed;
  ros::Time begin;
  ros::Time timer_start;
    
  // Waits for simulation time update
  ros::Time last_ros_time_;
  bool wait = true;
  while (wait)
  {
    last_ros_time_ = ros::Time::now();
    if (last_ros_time_.toSec() > 0){
      wait = false;
      timer_start = ros::Time::now();
      }
  }
  
  begin = ros::Time::now();
  last_pose_time = ros::Time::now();
   
while (ros::ok())
  {
  while (ros::ok())
  {
  
    //******Dead Reckoning Calculations Based on SfM pose, commands, time **************//
  
  double time_since_update = ros::Time::now().toSec() - last_pose_time.toSec();
  double cmd_xcomp, cmd_ycomp, x_calc_omvg, z_calc_omvg;
    
  cmd_xcomp = cmd.linear.x * cos(Pose::_Theta_dr) - cmd.linear.y * sin(Pose::_Theta_dr);
  cmd_ycomp = cmd.linear.x * sin(Pose::_Theta_dr) + cmd.linear.y * cos(Pose::_Theta_dr);
  Pose::_X_dr = Pose::_X_dr + time_since_update*cmd_xcomp; //total change in this direction for dead reckoning
  Pose::_Y_dr = Pose::_Y_dr + time_since_update*cmd_ycomp;
  Pose::_Theta_dr = Pose::_Theta_dr + time_since_update*cmd.angular.z;
    
  x_calc = Pose::_X_sfm + Pose::_X_dr;
  y_calc = Pose::_Y_sfm + Pose::_Y_dr;
  theta_calc = Pose::_Theta_sfm + Pose::_Theta_dr;
  
  x_calc_omvg = -1.0*y_calc;
  z_calc_omvg = x_calc;
  
  last_pose_time = ros::Time::now(); //reset timer
  
  std::cerr << std::endl << "Calculated x, z, theta (openMVG coordinates) = " << x_calc_omvg << ", " << z_calc_omvg << ", " << theta_calc << std::endl;
  
  //**********************************************************************************//
  
  /*
  if (!start_flag)
  {
  begin = ros::Time::now();
  start_flag = 1;
  }

  if (begin.toSec() == 0)
  {
  start_flag = 0;
  break;
  }
  */
  
// create and initialize calculation variables
  //double x, y, z, w, r, p, pi, xdiff, ydiff, proximity, prox_min;
  
/*
  x = Pose::X;
  y = Pose::Y;
  prox_min = 100;

// Perform Calculation and assign message object data values
  for (int i = 0; i < features.size(); i++) {

    xdiff = x - features[i].x;
    ydiff = y - features[i].y;
    proximity = sqrt(pow(xdiff,2) + pow(ydiff,2));
    if (proximity < prox_min){
        prox_min = proximity;
    } //if
    //ROS_ERROR_THROTTLE(5,"x = %f",x);
    //ROS_ERROR_THROTTLE(5,"y = %f",y);
    //ROS_ERROR("Proximity = %f",proximity);
    if (prox_min < 2.0){

        cmd.angular.z = -1/pow(prox_min,2); //begin turn if object is near
        //ROS_ERROR_THROTTLE(1,"Proximity triggered, cmd = %f",cmd.angular.z);
        //ROS_ERROR_THROTTLE(1,"cmd den = %f",pow(prox_min,10));
        break;
    } //if
    else {
        cmd.angular.z = 0.0; //reset to straight line motion
    }
  } //for loop
*/

  elapsed = ros::Time::now().toSec() - begin.toSec();

  if (elapsed < 5){
  cmd.linear.y = -0.001; //keep drcsim standing controller from starting
  }
  else if (elapsed < 35){
  cmd.linear.y = -0.15;
  cmd.angular.z = 0.01;
  }
  else if (elapsed < 65){
  cmd.linear.y = -0.1;
  cmd.angular.z = -0.015;
  }
  else if (elapsed > 65){
  
  //command calculations
  
  }
  else{ //add exit condition here if needed
  
  return 0;
  
  }
          
    pub_cmd.publish(cmd);

    ros::spinOnce();

    loop_rate.sleep();
    
  } //inner while loop
} //outer while loop

  return 0;
}
