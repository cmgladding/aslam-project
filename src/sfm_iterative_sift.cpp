//Based on https://github.com/openMVG/openMVG/blob/master/src/software/SfM/main_IncrementalSfM.cpp
// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include <cstdlib>

#include "openMVG/sfm/sfm.hpp"
#include "openMVG/system/timer.hpp"

//#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include "openMVG/image/image.hpp"
#include "openMVG/stl/split.hpp"

#include <ros/ros.h>
#include <ros/console.h>
#include <utility> //std::pair
#include <algorithm> //for vector search

#include "openMVG/matching_image_collection/Matcher_Regions_AllInMemory.hpp"
#include "openMVG/matching_image_collection/GeometricFilter.hpp"
#include "openMVG/matching_image_collection/F_ACRobust.hpp"

#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <sstream>

#include "openMVG/features/features.hpp"
#include "nonFree/sift/SIFT_describer.hpp"


using namespace std;
using namespace svg;
using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::sfm;

using namespace openMVG::features;
using namespace openMVG::image;
using namespace openMVG::matching_image_collection;
using namespace stlplus;

class ImageList {
public:
  ImageList();
  ~ImageList();
  //void setDirectory(const char* nm);
  //int countFiles();
  //void loadAllImages();
  //void loadImage(std::string s);
  //void generateFeatures();
  //void computeMatches();
  //void sequentialReconstruct();

//private:
  //string _directory;
  string _matches_full;
  string _matches_filtered;
  vector<string> _fnames;
  SfM_Data _sfm_data;
  unique_ptr<Regions> _regionsType;
  
  //string _sSfM_Data_Filename;
  string _sMatchesDir;
  string _sOutDir;
  pair <std::string,std::string> _initialPairString;
};

ImageList::ImageList() {
}

ImageList::~ImageList() {
}

/*
bool computeIndexFromImageNames(
  const SfM_Data & sfm_data,
  const std::pair<std::string,std::string>& initialPairName,
  Pair& initialPairIndex);
*/

/// From 2 given image file-names, find the two corresponding index in the View list
bool computeIndexFromImageNames(
  const SfM_Data & sfm_data,
  const std::pair<std::string,std::string>& initialPairName,
  Pair& initialPairIndex)
{
  if (initialPairName.first == initialPairName.second)
  {
    std::cerr << "\nInvalid image names. You cannot use the same image to initialize a pair." << std::endl;
    return false;
  }

  initialPairIndex = Pair(UndefinedIndexT, UndefinedIndexT);

  /// List views filenames and find the one that correspond to the user ones:
  std::vector<std::string> vec_camImageName;
  for (Views::const_iterator it = sfm_data.GetViews().begin();
    it != sfm_data.GetViews().end(); ++it)
  {
    const View * v = it->second.get();
    const std::string filename = stlplus::filename_part(v->s_Img_path);
    if (filename == initialPairName.first)
    {
      initialPairIndex.first = v->id_view;
    }
    else{
      if (filename == initialPairName.second)
      {
        initialPairIndex.second = v->id_view;
      }
    }
  }
  return (initialPairIndex.first != UndefinedIndexT &&
      initialPairIndex.second != UndefinedIndexT);
}

//---------------------------------------------------------------------------------------------

int main(int argc, char** argv)
{
 
  ros::init(argc, argv, "sfm_iterative");
  
  ImageList iml; //create imagelist structure to store data
    
  //Initialize filepath information
  //iml._sSfM_Data_Filename "/home/colin/aslam_project/SfM_PointCloudData/sfm_data.json"; //input file
  iml._sMatchesDir = "/home/colin/Documents/FilteredPhotoDump"; //directory holding match files (putative/geometric filtered)
  iml._sOutDir = "/home/colin/Documents/SfM_PointCloudData"; //selected output directory for point cloud
  
  //Define initial image pair filepaths
  iml._initialPairString.first = "Img00002.jpg"; 
  iml._initialPairString.second = "Img00010.jpg"; 
  
  iml._matches_full = stlplus::create_filespec(iml._sMatchesDir, "matches.putative.txt");
  iml._matches_filtered = stlplus::create_filespec(iml._sMatchesDir, "matches.f.txt");
    
  ros::NodeHandle n;
  
  //ros::Subscriber sfm_sub = n.subscribe("/multisense_sl/left/image_raw", 1000, sfmCB);
while (ros::ok())
  {
    while (ros::ok())
      {
  
  //openMVG::system::Timer timer_fileList;
  
  vector<string> file_list;
  iml._sfm_data.s_root_path = iml._sMatchesDir;
  file_list = stlplus::folder_files(iml._sMatchesDir);

  sort(file_list.begin(), file_list.end());
  
  //std::cerr << std::endl << "File listing time: " << timer_fileList.elapsed() << std::endl;
  
  ROS_INFO("File list updated, updating views for valid filenames...");
  
  //openMVG::system::Timer timer_view;

  for (vector<string>::const_iterator it = file_list.begin(); it != file_list.end(); it++)  {
    string which = *it; //"which" is the name of the file, whatever.jpg
    string imnm = stlplus::create_filespec(iml._sMatchesDir, which); //"imnm" is the whole filepath including filename
    
    //if img filename is not already in _fnames, this is our first time seeing it, so add a new view to SFM_DATA
    if (find(iml._fnames.begin(), iml._fnames.end(), imnm) == iml._fnames.end()){

    iml._fnames.push_back(imnm);
    
    ROS_INFO("Found new file! %s",imnm.c_str());
        
    //Contents of loadImage() from example code used below, note we aren't verifying a file exists:
  
  //Get latest view, set known intrinsics for that view
  double width, height, image_focal;
  double focal, ppx, ppy;

  Views& views = iml._sfm_data.views;
  Intrinsics& intrinsics = iml._sfm_data.intrinsics;
  shared_ptr<IntrinsicBase> intrinsic (NULL);

  if (GetFormat(which.c_str()) == openMVG::image::Unknown){
    //ROS_INFO("Format unknown for %s, skipping view loading.",which.c_str());
    //return;
    }
    else{
    
  //Known values for multisim_sl from DRCsim .ini files
  focal = 476.7030836014194; //focal length in pixels 
  width = 800.0; //sensor width in pixels
  height = 800.0; //sensor height in pixels
  
  ppx = width / 2.0; //image center point
  ppy = height / 2.0; //image center point

  printf("Image %s: %f x %f, Focal Length (pixels) %f\n", which.c_str(), width, height, focal);

  View v(imnm, views.size(), views.size(), views.size(), width, height);

  intrinsic = make_shared<Pinhole_Intrinsic_Radial_K3> (width, height, focal,
                                                        ppx, ppy, 0.0, 0.0, 0.0);
  intrinsics[v.id_intrinsic] = intrinsic;
  views[v.id_view] = std::make_shared<openMVG::sfm::View>(v);
  
  ROS_INFO("Loaded info for new view %s, skipping rest of files until next callback...",which.c_str());
  
  break; //only process one new view per CB
  
    } //if file format is known to openMVG
  } //if filename is new
} //for each file in the folder

  //std::cerr << std::endl << "View finding time: " << timer_view.elapsed() << std::endl;

  ROS_INFO("New view loaded if found, finding features if they don't exist already...");
  
  openMVG::system::Timer timer_feature;
  
// FOR NEW VIEWS, FIND FEATURES AND ADD TO SFM DATA - loadAllImages, loadImage from example code

  //Create image describer, generate _regionsType (similar to example code), assume stays the same
  //AKAZEParams params(AKAZEConfig(), AKAZE_MSURF);

  //unique_ptr<AKAZE_Image_describer> image_describer(new AKAZE_Image_describer(params, true)); //false assumes features always upright
  
  std::unique_ptr<Image_describer> image_describer;
  
  image_describer.reset(new SIFT_Image_describer(SiftParams(), 1));
    
  image_describer->Allocate(iml._regionsType);
  
  //Check every image and generate features if they don't exist yet (as in example code generateFeatures())
  image_describer->Set_configuration_preset(NORMAL_PRESET);

  for(Views::const_iterator iterViews = iml._sfm_data.views.begin(); iterViews != iml._sfm_data.views.end(); ++iterViews) {
  const View * view = iterViews->second.get();
  const string sFeat = stlplus::create_filespec(iml._sMatchesDir, stlplus::basename_part(view->s_Img_path), "feat");
  const string sDesc = stlplus::create_filespec(iml._sMatchesDir, stlplus::basename_part(view->s_Img_path), "desc");
  
    if (!stlplus::file_exists(sFeat)) {
      Image<unsigned char> imageGray;
      printf("Creating %s\n", sFeat.c_str());
      ReadImage(view->s_Img_path.c_str(), &imageGray);
      unique_ptr<Regions> regions;
      image_describer->Describe(imageGray, regions);
      image_describer->Save(regions.get(), sFeat, sDesc);
      ROS_INFO("Features found for %s",view->s_Img_path.c_str());
    }
    else {
      printf("Using existing features from %s\n", sFeat.c_str());
    }
  }
  
  std::cerr << std::endl << "Feature finding time: " << timer_feature.elapsed() << std::endl;

// GENERATE FEATURE MATCHES USING CONTIGUOUS PAIRS (as in example but using contiguous pairs)
//https://github.com/openMVG/openMVG/blob/master/src/software/SfM/main_ComputeMatches.cpp

  //ROS_INFO("Features found, finding contiguous pair matches...");

  float fDistRatio = 0.6f; // 0.8f dflt // Higher is stricter
  
  openMVG::system::Timer timer_matches;

  openMVG::matching::PairWiseMatches map_PutativesMatches;
  
  //***************************
  //Trying fast_cascade_hashing
  //***************************
  unique_ptr<Matcher_Regions_AllInMemory> collectionMatcher;
  collectionMatcher.reset(new Matcher_Regions_AllInMemory(fDistRatio, openMVG::ANN_L2));
  
  //std::unique_ptr<Matcher> collectionMatcher;
  //collectionMatcher.reset(new Cascade_Hashing_Matcher_Regions_AllInMemory(fDistRatio));
  
  //****************************
  
    openMVG::Pair_Set pairs;
    shared_ptr<Regions_Provider> regions_provider = make_shared<Regions_Provider>();
  if (!regions_provider->load(iml._sfm_data, iml._sMatchesDir, iml._regionsType)){
    ROS_ERROR("Regions provider load failed");
    //return;
    break; //break out of current while loop
    }
    
    //no longer checking if data loads here
    pairs = contiguousWithOverlap(iml._sfm_data.GetViews().size(),5); //find matches with three adjacent frames
    collectionMatcher->Match(iml._sfm_data,regions_provider, pairs, map_PutativesMatches);

    ofstream file_full(iml._matches_full); //changed file object name to avoid conflict
    if (file_full.is_open()){
      PairedIndMatchToStream(map_PutativesMatches, file_full);
      }
    file_full.close();

  openMVG::PairWiseMatches map_GeometricMatches;

std::unique_ptr<ImageCollectionGeometricFilter> filter_ptr(new ImageCollectionGeometricFilter(&iml._sfm_data, regions_provider));

  const double maxResidualError = 1.0; // dflt 1.0; // Lower is stricter

filter_ptr->Robust_model_estimation(GeometricFilter_FMatrix_AC(maxResidualError), map_PutativesMatches);
map_GeometricMatches = filter_ptr->Get_geometric_matches();

  ofstream file_filtered(iml._matches_filtered); //changed file object name to avoid conflict
  if (file_filtered.is_open()){
    PairedIndMatchToStream(map_GeometricMatches, file_filtered);
    }
  file_filtered.close();
  
  std::cerr << std::endl << "Feature matching time: " << timer_matches.elapsed() << std::endl;

//----------------------------------------------------------------------------------------
// SEQUENTIAL RECONSTRUCTION

  ROS_INFO("Matches updated, starting 3D reconstruction, report at %s", stlplus::create_filespec(iml._sOutDir, "Reconstruction_Report.html").c_str());
  
  // Get ready for output
  if (iml._sOutDir.empty())  {
    std::cerr << "\nIt is an invalid output directory" << std::endl;
    //return;
    break; //break out of current while loop
  }

  if (!stlplus::folder_exists(iml._sOutDir)){
    ROS_INFO("Output folder does not exist, creating it now.");
    if(!stlplus::folder_create(iml._sOutDir)){
    ROS_ERROR("Output folder creation failed!");
    }    
    }

  openMVG::system::Timer timer_reconstruction;
  SequentialSfMReconstructionEngine sfmEngine(
    iml._sfm_data,
    iml._sOutDir,
    stlplus::create_filespec(iml._sOutDir, "Reconstruction_Report.html"));
  
  // Load feature and match providers from data
  shared_ptr<Features_Provider> feats_provider = std::make_shared<Features_Provider>();
  shared_ptr<Matches_Provider> matches_provider = std::make_shared<Matches_Provider>();

  if (!feats_provider->load(iml._sfm_data, iml._sMatchesDir, iml._regionsType)){
    ROS_ERROR("Features provider load failed");
    //return;
    break; //break out of current while loop
    }
    
  if (!matches_provider->load(iml._sfm_data, iml._matches_filtered)){
    ROS_ERROR("Matches provider load failed");
    //return;
    break; //break out of current while loop
    }

  // Configure the features_provider & the matches_provider
  sfmEngine.SetFeaturesProvider(feats_provider.get());
  sfmEngine.SetMatchesProvider(matches_provider.get());

  // Configure reconstruction parameters
  bool bRefineIntrinsics = false; //assume intrinsics are constant, correct for BA purposes
  sfmEngine.Set_bFixedIntrinsics(!bRefineIntrinsics);
  int i_User_camera_model = PINHOLE_CAMERA_RADIAL3; //Define camera model
  sfmEngine.SetUnknownCameraType(EINTRINSIC(i_User_camera_model));

  // Handle Initial pair parameter
  if (!iml._initialPairString.first.empty() && !iml._initialPairString.second.empty())
  {
    Pair initialPairIndex;
    
    //Return if initial pair names are not found in data so far
    if(!computeIndexFromImageNames(iml._sfm_data, iml._initialPairString, initialPairIndex))
    {
        std::cerr << "Could not find the initial pairs <" << iml._initialPairString.first 
          <<  ", " << iml._initialPairString.second << ">!\n";
      //return;
      break; //break out of current while loop
    }
    sfmEngine.setInitialPair(initialPairIndex);
  }

sfmEngine.Process();
  //if (sfmEngine.Process())
  //{
  
    ROS_INFO("Reconstruction successful, saving to file...");
    


    //std::cout << "...Generating SfM_Report.html" << std::endl;
    //Generate_SfM_Report(sfmEngine.Get_SfM_Data(), stlplus::create_filespec(iml._sOutDir, "SfMReconstruction_Report.html"));
      
    ROS_INFO("Reconstruction report saved at %s", stlplus::create_filespec(iml._sOutDir, "SfMReconstruction_Report.html").c_str());

    //-- Export to disk computed scene (data & visualizable results)
    std::cout << "...Export SfM_Data to disk." << std::endl;
    Save(sfmEngine.Get_SfM_Data(),
      stlplus::create_filespec(iml._sOutDir, "sfm_data", ".json"),
      openMVG::sfm::ESfM_Data(ALL));

    ROS_INFO("SfM data saved to %s", stlplus::create_filespec(iml._sOutDir, "sfm_data", ".json").c_str());

    Save(sfmEngine.Get_SfM_Data(),
      stlplus::create_filespec(iml._sOutDir, "cloud_and_poses", ".ply"),
      openMVG::sfm::ESfM_Data(ALL));
      
    ROS_INFO("Point cloud saved to %s", stlplus::create_filespec(iml._sOutDir, "cloud_and_poses", ".ply").c_str());
  //}
  
  std::cerr << std::endl << " Total Ac-Sfm took (s): " << timer_reconstruction.elapsed() << std::endl;
ROS_INFO("Reconstruction saved to %s", stlplus::create_filespec(iml._sOutDir, "cloud_and_poses", ".ply").c_str());
  
  }//nested while loop
}//second while loop (so we can use break statements without ending main()
  //ros::spin();

}
