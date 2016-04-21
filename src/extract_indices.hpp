#include <iostream>
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



float findAngleLinePlane(std::vector<float> pCoeffs,std::vector<float> lCoeffs);


bool isVertical(std::vector<float> pCoeffs);


void setNormals(pcl::PointCloud<pcl::Normal>::Ptr normals,int nPoints,std::vector<float> norm);


float euclidean(pcl::PointXYZ p1,pcl::PointXYZ p2);


int sgnn(float x);


void findEndPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p,pcl::PointCloud<pcl::PointXYZ>::Ptr endPoints);

void findNormals(pcl::PointCloud<pcl::Normal>::Ptr normals,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p);


void findLinNormals(pcl::PointCloud<pcl::Normal>::Ptr normals,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p);
//int main (int argc, char** argv)

bool getM(std::vector<pcl::PointCloud <pcl::PointXYZ>::Ptr> &,std::vector<pcl::ModelCoefficients::Ptr> &);
