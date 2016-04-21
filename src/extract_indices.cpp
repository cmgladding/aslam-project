#include <iostream>
#include <vector>
#include <pcl/io/ply_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/centroid.h>
#include "extract_indices.hpp"
#include <pcl/sample_consensus/sac_model_parallel_plane.h>
//int main (int argc, char** argv)

//void getMap(std::vector<pcl::PointCloud <pcl::PointXYZ>::Ptr> &verticalEndpoints,std::vector<pcl::ModelCoefficients::Ptr> &verticalCoef);



float findAngleLinePlane(std::vector<float> pCoeffs,std::vector<float> lCoeffs)
{
	float q = pCoeffs[0]*lCoeffs[0]+pCoeffs[1]*lCoeffs[1]+pCoeffs[2]*lCoeffs[2];
	float p = (sqrt(pow(lCoeffs[0],2)+pow(lCoeffs[1],2)+pow(lCoeffs[2],2))*sqrt(pow(pCoeffs[0],2)+pow(pCoeffs[1],2)+pow(pCoeffs[2],2)));
	
	float t = fabs(q)/p;
    float y = asin(t);	
	//return (asin(abs(pCoeffs[0]*lCoeffs[0]+pCoeffs[1]*lCoeffs[1]+pCoeffs[2]*lCoeffs[2])/(sqrt(pow(lCoeffs[0],2)+pow(lCoeffs[1],2)+pow(lCoeffs[2],2))*sqrt(pow(pCoeffs[0],2)+pow(pCoeffs[1],2)+pow(pCoeffs[2],2)))));
//	std::cout<<"\tq="<<q<<"\tp="<<p<<"\tv="<<t<<"\tThe="<<y<<std::endl;
    return(y); 
}

bool isVertical(std::vector<float> pCoeffs)
{
	double pi = 3.14;
	static const float arr[] = {0,1,0};
	std::vector<float> lCoeffs (arr,arr+sizeof(arr)/sizeof(arr[0]));
	float angle = findAngleLinePlane(pCoeffs,lCoeffs);
	float thresh = 0,tol = 1;
	//std::cout<<"angle="<<angle<<"a="<<pCoeffs[0]<<"b="<<pCoeffs[1]<<"c="<<pCoeffs[2]<<"d="<<pCoeffs[3]<<std::endl;
	return (fabs(angle - thresh) < tol);
}

void setNormals(pcl::PointCloud<pcl::Normal>::Ptr normals,int nPoints,std::vector<float> norm)
{
	normals->width = nPoints;
	normals->height = 1;
	normals->points.resize(normals->width*normals->height);
	for (size_t i=0;i < nPoints; i++)
	{
	//	(normals->points[i])->Normal();
	}
}

float euclidean(pcl::PointXYZ p1,pcl::PointXYZ p2)
{
	return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}

int sgnn(float x)
{
	if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

void findEndPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p,pcl::PointCloud<pcl::PointXYZ>::Ptr endPoints)
{
	//pcl::PointCloud<pcl::PointXYZ>::Ptr endPoints;
	endPoints ->width = 2;
	endPoints ->height = 1;
	endPoints ->points.resize(endPoints ->width*endPoints ->height);
	
	pcl::PointXYZ c;
	c.x = 0.0f;
	c.y = 0.0f;
	c.z = 0.0f;

	for(size_t i=0;i<cloud_p->points.size();i++)
	{
		c.x += cloud_p->points[i].x;
		c.y += cloud_p->points[i].y;
		c.z += cloud_p->points[i].z;
	}
	c.x /= cloud_p->points.size();
	c.y /= cloud_p->points.size();
	c.z /= cloud_p->points.size();
	
	
	float maxDist1 = -1;
	int maxIndex1 = 0;
	float maxDist2 = -1;
	int maxIndex2 = 0;
	float t1 = c.x-cloud_p->points[0].x;
	float t2 = c.y-cloud_p->points[0].y;
	int s1 = sgnn(t1);
	int s2 = sgnn(t2);
	
	for(size_t i=0;i<cloud_p->points.size();i++)
	{   
		float curDist = euclidean(c,cloud_p->points[i]);
	//	std::cout<<"\t"<<maxDist2<<"\t"<<curDist<<"\t"<<maxIndex2<<"\t"<<i<<"\t"<<c<<"\t"<<std::endl; 
		//std::cout<<s1<<"\ta"<<s2<<"\tb"<<maxIndex1<<"\tvv"<<maxIndex2<<"\tii"<<sgnn(c.y - cloud_p->points[i].y)<<std::endl;
	/*		if ( (s1 == sgnn(c.x-cloud_p->points[i].x)) && (s2 == sgnn(c.y - cloud_p->points[i].y)))
		{
			std::cout<<"-------------------------"<<"\t"<<i<<"\t"<<maxDist1<<"\t"<<curDist<<std::endl;
			if (maxDist1 < curDist)
			{
			maxDist1 = curDist;
			maxIndex1 = i;
		}
		}
		else
		{
			if (maxDist2 < curDist)
			{
			maxDist2 = curDist;
			maxIndex2 = i;
		}
	}*/
//	endPoints->points[0].x = 2*c.x - cloud_p->points[maxIndex2].x;
//	endPoints->points[0].y = 2*c.y - cloud_p->points[maxIndex2].y;
	
//	endPoints->points[0] = cloud_p->points[maxIndex1];
	
		if (maxDist2 < curDist)
			{
			maxDist2 = curDist;
			maxIndex2 = i;
		}
	}
	endPoints->points[0].x = 2*c.x - cloud_p->points[maxIndex2].x;
	endPoints->points[0].y = 2*c.y - cloud_p->points[maxIndex2].y;
	endPoints->points[1] = cloud_p->points[maxIndex2];
	
	
	
}







bool getM(std::vector<pcl::PointCloud <pcl::PointXYZ>::Ptr> &verticalEndpoints,std::vector<pcl::ModelCoefficients::Ptr> &verticalCoef)
{
 //* load the file
 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
if (pcl::io::loadPLYFile<pcl::PointXYZ> ("/home/colin/Documents/SfM_PointCloudData/cloud_and_poses.ply", *cloud_filtered) == -1)
 {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return false ;
  }
  // Fill in the cloud data
  
  /*cloud_filtered->width  = 2000;
  cloud_filtered->height = 1;
  cloud_filtered->points.resize (cloud_filtered->width * cloud_filtered->height);

 
   for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
  {
    cloud_filtered->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_filtered->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_filtered->points[i].x = float(i%10) ;
    
  }*/
   bool status = false;
  // Convert to the templated PointCloud
  //pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

  // Write the downsampled version to disk
//  pcl::PCDWriter writer;
  
  //writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
 pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
 seg.setOptimizeCoefficients (true);
  // Mandatory
// seg.setModelType (pcl::SACMODEL_PLANE);
   seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);

    Eigen::Vector3f axis = Eigen::Vector3f(0.0,1.0,0.0);

 seg.setAxis(axis);
    seg.setEpsAngle(  15.0f * (M_PI/180.0f) );
 // seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.01);
  
    
  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
 //atic const double arr[] = {0,1,0};
  //std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> verticalEndpoints;
   //std::vector<pcl::ModelCoefficients::Ptr> verticalCoef;

  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  // While 30% of the original cloud is still there
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
	inliers->indices.clear();  
	coefficients->values.clear();
    // Segment the largest planar component from the remaining cloud
  //  std::cout<<"aa"<<cloud_filtered->points.size()<<std::endl;

 /*   pcl::SampleConsensusModelParallelPlane<pcl::PointXYZ> model (cloud_filtered);
model.setAxis (Eigen::Vector3f (0.0, 1.0, 0.0));
model.setEpsAngle (15.0f * (M_PI/180.0f));
std::vector<int> ind;
for (int j=0;j<cloud_filtered.size();j++)
{
	ind.pushback(j);
}
std::vector<float> cv;
Eigen::VectorXf vv;
std::cout<<"bb"<<cloud_filtered->points.size()<<std::endl;
bool a = model.computeModelCoefficients(ind,vv);
model.getInliers(*inliers);

for (size_t j=0;j<vv.size();j++)
{
coefficients->values.push_back(vv[j]);
	
}*/
    
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
       
     
    //std::cout<<"IsVerical"<<isVertical(coefficients->values)<<std::endl;

    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    /*
     float tmp=0,sd=0;
    for (size_t j = 0; j < cloud_p->points.size (); ++j)
  {
    
    tmp += cloud_p->points[j].y ;
    
  }
   tmp /= cloud_p->points.size();
    for (size_t j = 0; j < cloud_p->points.size (); ++j)
  {
    
    sd += pow((tmp-cloud_p->points[j].y),2) ;
    
  }
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points =" <<sd<<"p="<<tmp<<"d="<< std::endl;
   */

  //  std::stringstream ss;
    //ss << "table_scene_lms400_plane_" << i << ".pcd";
  //  writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);
    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    if (inliers->indices.size()!=0 && isVertical(coefficients->values))
    {
		
	 
	  status = true; 
	  /*
	  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>); 	
	  	
	  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne; 
      ne.setInputCloud (cloud_p); 
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ()); 
      ne.setSearchMethod (tree); 
      ne.setRadiusSearch (1); 
      ne.compute (*normals);
	 
	  pcl::PointCloud<pcl::Boundary> boundaries;
	  pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
	  est.setInputCloud(cloud_p);
	  est.setInputNormals(normals);
	  est.setRadiusSearch (10);  
	  est.setSearchMethod (typename pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
      est.compute (boundaries);
     int count = 0;
          for (size_t j = 0; j < boundaries.points.size (); ++j)
  {
    
    if (boundaries.points[j].boundary_point == 1)
    {
		
    count++;
}
    
  }*/
   pcl::PointCloud<pcl::PointXYZ>::Ptr line_points (new pcl::PointCloud<pcl::PointXYZ>);
      
 // line_points->width = count;
  line_points->width = cloud_p->points.size();
  line_points->height=1;
  line_points->points.resize(line_points->width*line_points->height);
  
  
  //for (size_t j = 0; j < boundaries.points.size (); ++j)
  for (size_t j=0;j<cloud_p->points.size();j++)
  {
	  
	 
	 line_points->points[j].x=cloud_p->points[j].x;
	line_points->points[j].y=cloud_p->points[j].z;	
	line_points->points[j].z= 0;
    /*
    if (boundaries.points[j].boundary_point == 1)
    {
	line_points->points[k].x=cloud_p->points[j].x;
	line_points->points[k].y=cloud_p->points[j].z;	
	line_points->points[k].z= 0;
	k++;
   
}*/
  }
  
   pcl::ModelCoefficients::Ptr lincoefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr lininliers (new pcl::PointIndices ());
   pcl::SACSegmentation<pcl::PointXYZ> linseg;
   
   linseg.setModelType (pcl::SACMODEL_LINE);
  linseg.setMethodType (pcl::SAC_RANSAC);
  linseg.setMaxIterations (10000);
  linseg.setDistanceThreshold (0.01);
      linseg.setInputCloud (line_points);
    linseg.segment (*lininliers, *lincoefficients);
  
     
      pcl::PointCloud<pcl::PointXYZ>::Ptr endPoints (new pcl::PointCloud<pcl::PointXYZ>);
      findEndPoints(line_points,endPoints);     
  
	  verticalEndpoints.push_back(endPoints);
	  verticalCoef.push_back(lincoefficients);
	
    }
      

    cloud_filtered.swap (cloud_f);
    i++;
   
    
  }
   

  
  return(status);
}


