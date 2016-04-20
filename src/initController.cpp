#include <cmath>
#include "ros/ros.h"
#include <ros/console.h>
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include <std_msgs/Float64.h>
#include <vector>

/*
class Pose{
public:

  // 2D robot pose in the plane
    static double X;
    static double Y;

  Pose();
  ~Pose();

}; // class

  // Initialize pose values (in case we try to calculate before calling CB)
  double Pose::X = 0.0;
  double Pose::Y = 0.0;

struct feature_point { // feature point detected by the system
  double x;
  double y;
  double z;

};

// Read data from gazebo and update public data values
void readPose(geometry_msgs::Point position)
{
  Pose::X = position.x;
  Pose::Y = position.y;
}

void poseCB(const gazebo_msgs::ModelStates::ConstPtr& msg)
{

  // Figure out which array element is for atlas
  int index_max = msg->name.size();
  int index_atlas = -1;
  std::string atlas_name = "atlas";
  std::string index_name;
  
  for (int index=0;index<index_max;index++){
    index_name = msg->name[index];

    if (index_name == atlas_name){
       index_atlas = index;
    }
  }
  
  if (index_atlas != -1){
    readPose(msg->pose[index_atlas].position);
  }
}
*/

int main(int argc, char **argv)
{

  ros::init(argc, argv, "initController");

  ros::NodeHandle n;

  // Subscribe to the arrays of actual model poses and twists published by gazebo
  //ros::Subscriber sub_model_pose = n.subscribe<gazebo_msgs::ModelStates>("gazebo/model_states", 1, poseCB);

  // Consider putting these all into a single custom message type
  ros::Publisher pub_cmd = n.advertise<geometry_msgs::Twist>("atlas/cmd_vel", 1000);

  geometry_msgs::Twist cmd; //create message object

  ros::Rate loop_rate(10);
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
 
  features.push_back(feature_point());
  //generate synthetic wall 2 points
  features[length+1].x = 8.0;
  features[length+1].y = 10.0*length/num_points - 9.0;
  features[length+1].z = 0.0;

  features.push_back(feature_point());
  //generate synthetic wall 3 points
  features[length+2].x = 10.0*length/num_points - 2.0;
  features[length+2].y = -9.0;
  features[length+2].z = 0.0;

  features.push_back(feature_point());
  //generate synthetic wall 4 points
  features[length+3].x = -2.0;
  features[length+3].y = 10.0*length/num_points -9.0;
  features[length+3].z = 0.0;

  } //for loop - wall generation
*/

// Use publish() function to send message objects
  cmd.linear.x = 0.0; // 0.25 gives a pretty slow motion //initialize forward velocity
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;
  cmd.angular.z = 0.0; //initialize turning velocity (assume positive z axis points up)
  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;
  
  bool start_flag = 0;
  ros::Time begin;
while (ros::ok())
  {
while (ros::ok())
  {
  while (ros::ok())
  {
  
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
  
// create and initialize calculation variables
  //double x, y, z, w, r, p, pi, xdiff, ydiff, proximity, prox_min;
  double pi = 3.14159265359;
  
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

  cmd.linear.x = 0.0;
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;
  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;
  cmd.angular.z = 0.0;

  //ros::Duration elapsed = ros::Time::now() - begin;
  double elapsed = ros::Time::now().toSec() - begin.toSec();
  //ROS_ERROR("Time is...%f",elapsed);

  if (elapsed < 15){
  cmd.linear.y = -0.5;
  //ROS_ERROR("Time < 10");
  }
  else if (elapsed < 60){
  cmd.angular.z = 0.15;
  //ROS_ERROR("Time < 15");
  }
  else{
  //ROS_ERROR("Exiting...%f",elapsed);
  return 0;
  }
        
    pub_cmd.publish(cmd);

    //ros::spinOnce();

    loop_rate.sleep();
  }
}
}
  return 0;
}
