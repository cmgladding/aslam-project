//http://docs.opencv.org/2.4/doc/tutorials/imgproc/gausian_median_blur_bilateral_filter/gausian_median_blur_bilateral_filter.html
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
//#include <opencv2/imgcodecs.hpp>

using namespace std;
using namespace cv;

int MAX_KERNEL_LENGTH = 5;
int IMG_COUNT = 1;
int CATCHUP_STATE = 0; //1 if we need to catch up on images (stop processing)
int INIT_STATE = 0; //1 if initial pair has been processed

ros::Time timer_start;

Mat src;
Mat dst;

void catchupCB(const std_msgs::Float64ConstPtr& msg)
{
  CATCHUP_STATE = int (msg->data);
}

void initCB(const std_msgs::Float64ConstPtr& msg)
{
  INIT_STATE = int (msg->data);
}

void filterCB(const sensor_msgs::ImageConstPtr& msg)
{

double elapsed = ros::Time::now().toSec() - timer_start.toSec();

//if (!INIT_STATE || (!CATCHUP_STATE && elapsed > 6.0)){ //once initialized, only process new images if we were recently caught up in processing them
if (!CATCHUP_STATE && elapsed > 5.5){ //only process new images if we were recently caught up in processing them

  string filename;
  ostringstream convert;

  cv_bridge::CvImagePtr cv_ptr;
  
  //convert to openCV image
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  
  //apply filter (process cv_ptr->image using OpenCV)
  for ( int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 ){
        GaussianBlur( cv_ptr->image, dst, Size( i, i ), 0, 0 );
        }
  
  string filepath;
  if (IMG_COUNT < 10){
    filepath = "/home/colin/Documents/FilteredPhotoDump/Img0000";
    }
  else if (IMG_COUNT < 100){
    filepath = "/home/colin/Documents/FilteredPhotoDump/Img000";
    }
  else if (IMG_COUNT < 1000){
    filepath = "/home/colin/Documents/FilteredPhotoDump/Img00";
    }
  else if (IMG_COUNT < 10000){
    filepath = "/home/colin/Documents/FilteredPhotoDump/Img0";
    }
  else if (IMG_COUNT < 100000){
    filepath = "/home/colin/Documents/FilteredPhotoDump/Img";
    }
  else if (IMG_COUNT >= 100000){
    ROS_ERROR("Error: Image count too high!");
    return;
    }
  
  //io - save jpg to file
  convert << filepath << IMG_COUNT << ".jpg";
  imwrite( convert.str(), dst);
  
  IMG_COUNT++;
  
  timer_start = ros::Time::now(); //restart image timer
  
  } //if not in catchup mode

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imFilter");

  ros::NodeHandle n;
  
  ros::Subscriber im_sub = n.subscribe("/multisense_sl/left/image_raw", 1000, filterCB);
  ros::Subscriber catchup_sub = n.subscribe("catchup_state", 1000, catchupCB);
  ros::Subscriber init_sub = n.subscribe("initialization_state", 1000, initCB);
  
    // Waits for simulation time update.
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
  
  timer_start = ros::Time::now(); //take first photo right away
  
  ros::spin();

  return 0;
}
