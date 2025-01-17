#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include "extract_indices.cpp"

#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointClouds;


//int main()
int main(int argc, char** argv)
{
  	
  ros::init (argc, argv, "pub_pcl");
  
  
  ros::NodeHandle nh;
 
  ros::Publisher pub = nh.advertise<PointClouds> ("points2", 1);
 

   
  PointClouds::Ptr msg (new PointClouds);
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> verticalEndpoints;
	std::vector<pcl::ModelCoefficients::Ptr> verticalCoef;
  msg->header.frame_id = "some_tf_frame";
  msg->height = msg->width = 1;
  //msg->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));

  ros::Rate loop_rate(4);
  while (nh.ok())
  {
	
	msg->points.clear();
	verticalEndpoints.clear();
	verticalCoef.clear();
	bool status = getM(verticalEndpoints,verticalCoef);
	if (status)
	{
	
	for (int i=0;i<verticalEndpoints.size();i++)
	{
		
	  msg->points.push_back(verticalEndpoints[i]->points[0]);
	  msg->points.push_back(verticalEndpoints[i]->points[1]);
	  
	}
}
    else
    {

		msg->points.push_back(pcl::PointXYZ(0,0,0));
	}
	

    msg->header.stamp = ros::Time::now().toNSec();
   
    pub.publish (msg);
   

    ros::spinOnce ();
    

    //loop_rate.sleep ();
    
  }
}
