#include <cmath>
#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include <std_msgs/Float64.h>
#include <vector>

class Rotation{
public:

  // Quaternion parameters
  // Other convention - w is x0, x is x1, y is x2, z is x3
    static double X;
    static double Y;
    static double Z;
    static double W;

  Rotation();
  ~Rotation();

}; // class

  // Initialize quaternion values (in case we try to calculate before calling CB)
  double Rotation::W = 1.0;
  double Rotation::X = 0.0;
  double Rotation::Y = 0.0;
  double Rotation::Z = 0.0;

// Read data from gazebo in quaternion format and update public data values
void readOrientation(geometry_msgs::Quaternion orientation)
{
  Rotation::X = orientation.x;
  Rotation::Y = orientation.y;
  Rotation::Z = orientation.z;
  Rotation::W = orientation.w;
}

void rotationCB(const gazebo_msgs::ModelStates::ConstPtr& msg)
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
    readOrientation(msg->pose[index_atlas].orientation);
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "quat2taitbryan");

  ros::NodeHandle n;

  // Subscribe to the arrays of actual model poses and twists published by gazebo
  ros::Subscriber sub_model_states = n.subscribe<gazebo_msgs::ModelStates>("gazebo/model_states", 1, rotationCB);

  // Consider putting these all into a single custom message type
  ros::Publisher pub_roll = n.advertise<std_msgs::Float64>("roll", 1000);
  ros::Publisher pub_pitch = n.advertise<std_msgs::Float64>("pitch", 1000);
  ros::Publisher pub_yaw = n.advertise<std_msgs::Float64>("yaw", 1000);

  ros::Rate loop_rate(200);

  while (ros::ok())
  {

// Create message objects
  std_msgs::Float64 roll, pitch, yaw;

// create and initialize calculation variables
  double x, y, z, w, r, p, pi;
  pi = 3.14159265359;
  w = Rotation::W;
  x = Rotation::X;
  y = Rotation::Y;
  z = Rotation::Z;

// Perform Calculation and assign message object data values
// Reference http://www.sedris.org/wg8home/Documents/WG80485.pdf
  roll.data = atan2((y*z + w*x),0.5 - (pow(x,2.0) + pow(y,2.0)));
  pitch.data = asin(-2*(x*z - w*y));
  yaw.data = atan2((x*y + w*z),0.5 - (pow(y,2.0) + pow(z,2.0)));

// Use publish() function to send message objects
  pub_roll.publish(roll);
  pub_pitch.publish(pitch);
  pub_yaw.publish(yaw);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
