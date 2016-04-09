//http://docs.opencv.org/2.4/doc/tutorials/imgproc/gausian_median_blur_bilateral_filter/gausian_median_blur_bilateral_filter.html
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
//#include <opencv2/imgcodecs.hpp>

using namespace std;
using namespace cv;

int MAX_KERNEL_LENGTH = 5;
int IMG_COUNT = 1;

Mat src;
Mat dst;

void filterCB(const sensor_msgs::ImageConstPtr& msg)
{

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
  
  //io - save jpg to file
  convert << "/home/colin/Documents/FilteredPhotoDump/Img" << IMG_COUNT << ".jpg";
  imwrite( convert.str(), dst);
  
  IMG_COUNT++;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imFilter");

  ros::NodeHandle n;
  
  ros::Subscriber im_sub = n.subscribe("/multisense_sl/left/image_raw", 1000, filterCB);

  ros::spin();

  return 0;
}