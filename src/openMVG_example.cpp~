//http://creativecommons.org/licenses/by-sa/4.0/
//http://hackingonspace.blogspot.com/2015/09/using-openmvg-library.html

#include <iostream>
#include "openMVG/sfm/sfm.hpp"
//#include "openMVG/exif/exif_IO_EasyExif.hpp"
#include "openMVG/image/image.hpp"
#include "openMVG/stl/split.hpp"

#include "openMVG/matching_image_collection/Matcher_Regions_AllInMemory.hpp"
#include "openMVG/matching_image_collection/GeometricFilter.hpp"
#include "openMVG/matching_image_collection/F_ACRobust.hpp"
#include "openMVG/sfm/sfm.hpp"
#include <ros/console.h>

using namespace std;
using namespace openMVG::sfm;
using namespace openMVG::features;
using namespace openMVG::image;
using namespace openMVG::cameras;
//using namespace openMVG::exif;
using namespace openMVG::matching_image_collection;
using namespace stlplus;

class ImageList {
public:
  ImageList();
  ~ImageList();
  void setDirectory(const char* nm);
  int countFiles();
  void loadAllImages();
  void loadImage(std::string s);
  void generateFeatures();
  void computeMatches();
  void sequentialReconstruct();

private:
  string _directory;
  string _matches_full;
  string _matches_filtered;
  vector<string> _fnames;
  SfM_Data _sfm_data;
  unique_ptr<Regions> _regionsType;
};

ImageList::ImageList() {
}

ImageList::~ImageList() {
}

void ImageList::setDirectory(const char* nm) {
vector<string> file_list;
  _directory = nm;
  _sfm_data.s_root_path = nm;
  file_list = stlplus::folder_files(_directory);

  sort(file_list.begin(), file_list.end());

  for (vector<string>::const_iterator it = file_list.begin(); it != file_list.end(); it++)  {
    string which = *it;
    string imnm = stlplus::create_filespec(_directory, which);
    _fnames.push_back(imnm);
  }
  //_matches_full = string(_directory + "/matches.putative.txt");
  //_matches_filtered = string(_directory + "/matches.f.txt");
  _matches_full = stlplus::create_filespec(_directory, "matches.putative.txt");
  _matches_filtered = stlplus::create_filespec(_directory, "matches.f.txt");
}

int ImageList::countFiles() {
return _fnames.size();
}

void ImageList::loadAllImages() {
  for ( vector<string>::const_iterator iter_image = _fnames.begin(); iter_image != _fnames.end(); iter_image++ ) {
    string which = *iter_image;
    loadImage(which);
  }
return;
}

void ImageList::loadImage(string which) {
double width, height, image_focal;
double focal, ppx, ppy;

Views& views = _sfm_data.views;
Intrinsics& intrinsics = _sfm_data.intrinsics;
shared_ptr<IntrinsicBase> intrinsic (NULL);

  if (GetFormat(which.c_str()) == openMVG::image::Unknown){
    //ROS_ERROR("Format unknown");
    return;
    }
    
  //Exif data extraction not required (not compatible with ROS image outputs anyway)
/*
  //Pointer must be of type Exif_IO
  unique_ptr<Exif_IO_EasyExif> exifReader(new Exif_IO_EasyExif());
  //ROS_ERROR("exifReader created");
  //std::unique_ptr<Exif_IO> exifReader(new Exif_IO_EasyExif());
  if (!exifReader->open(which)){
    //ROS_ERROR("exifReader Failed");
    return;
    }


  image_focal = static_cast<double>(exifReader->getFocal());
  width = static_cast<double>(exifReader->getWidth());
  height = static_cast<double>(exifReader->getHeight());
  
  ppx = width / 2.0; //image center point
  ppy = height / 2.0; //image center point
    
  if (image_focal == 0)
    return;

  const double ccdw = 5.75; // Cheap Samsung S890
  //const double ccdw = 4.62;// Also Cheap Canon SX410-IS
  //const double ccdw = 7.39; // Nicer Canon G9
  //const double ccdw = 35.9; // Very Nice Sony DSLR/A850
  focal = std::max (width, height) * image_focal / ccdw;

  printf("Image %s: %f x %f, Focal Length (mm) %f, Focal Length (pixels) %f\n", which.c_str(), width, height, image_focal,focal);
  */

  //Known values for multisim_sl from DRCsim .ini files
  focal = 476.7030836014194; //focal length in pixels 
  width = 800.0; //sensor width in pixels
  height = 800.0; //sensor height in pixels
  
  ppx = width / 2.0; //image center point
  ppy = height / 2.0; //image center point

  printf("Image %s: %f x %f, Focal Length (pixels) %f\n", which.c_str(), width, height, focal);


  View v(which, views.size(), views.size(), views.size(), width, height);

  intrinsic = make_shared<Pinhole_Intrinsic_Radial_K3> (width, height, focal,
                                                        ppx, ppy, 0.0, 0.0, 0.0);
  intrinsics[v.id_intrinsic] = intrinsic;
  views[v.id_view] = std::make_shared<openMVG::sfm::View>(v);
}

void ImageList::generateFeatures(void) {
AKAZEParams params(AKAZEConfig(), AKAZE_MSURF);
unique_ptr<AKAZE_Image_describer> image_describer(new AKAZE_Image_describer(params, true));

  image_describer->Allocate(_regionsType);
  image_describer->Set_configuration_preset(NORMAL_PRESET);

  for(Views::const_iterator iterViews = _sfm_data.views.begin(); iterViews != _sfm_data.views.end(); ++iterViews) {
  const View * view = iterViews->second.get();
  const string sFeat = stlplus::create_filespec(_directory, stlplus::basename_part(view->s_Img_path), "feat");
  const string sDesc = stlplus::create_filespec(_directory, stlplus::basename_part(view->s_Img_path), "desc");
  
    if (!stlplus::file_exists(sFeat)) {
      Image<unsigned char> imageGray;
      printf("Creating %s\n", sFeat.c_str());
      ReadImage(view->s_Img_path.c_str(), &imageGray);
      unique_ptr<Regions> regions;
      image_describer->Describe(imageGray, regions);
      image_describer->Save(regions.get(), sFeat, sDesc);
    }
    else {
      printf("Using existing features from %s\n", sFeat.c_str());
    }
  }
}

void ImageList::computeMatches() {
float fDistRatio = 0.6f; // 0.6f dflt // Higher is stricter

openMVG::matching::PairWiseMatches map_PutativesMatches;
vector<string> vec_fileNames;
vector<pair<size_t, size_t> > vec_imagesSize;

  for (Views::const_iterator iter = _sfm_data.GetViews().begin(); iter != _sfm_data.GetViews().end(); ++iter) {
    const View * v = iter->second.get();
    vec_fileNames.push_back(stlplus::create_filespec(_sfm_data.s_root_path, v->s_Img_path));
    vec_imagesSize.push_back(make_pair( v->ui_width, v->ui_height) );
  }

  unique_ptr<Matcher_Regions_AllInMemory> collectionMatcher;
  collectionMatcher.reset(new Matcher_Regions_AllInMemory(fDistRatio, openMVG::ANN_L2));

/*
//Sample code section updated to conform to latest library updates
//https://github.com/openMVG/openMVG/commit/6029189456152aa5dd4de0a822da5d2c06835a36

//no function loadData under collectionmatcher
  if (collectionMatcher->loadData(_regionsType, vec_fileNames, _directory)) {
    openMVG::Pair_Set pairs;
    pairs = openMVG::exhaustivePairs(_sfm_data.GetViews().size());
//no function Match under collectionmatcher
    collectionMatcher->Match(vec_fileNames, pairs, map_PutativesMatches);
    ofstream file(_matches_full);
    if (file.is_open())
      PairedIndMatchToStream(map_PutativesMatches, file);
    file.close();
  }*/
  
    openMVG::Pair_Set pairs;
    shared_ptr<Regions_Provider> regions_provider = make_shared<Regions_Provider>(); //added
  if (!regions_provider->load(_sfm_data, _directory, _regionsType)){ //added
    ROS_ERROR("Regions provider load failed");
    return;
    }
    
    //no longer checking if data loads
    pairs = openMVG::exhaustivePairs(_sfm_data.GetViews().size());
    collectionMatcher->Match(_sfm_data,regions_provider, pairs, map_PutativesMatches); //revised

    ofstream file_full(_matches_full); //changed file object name to avoid conflict
    if (file_full.is_open()){
      PairedIndMatchToStream(map_PutativesMatches, file_full);
      }
    file_full.close();

  shared_ptr<Features_Provider> feats_provider = make_shared<Features_Provider>();

  if (!feats_provider->load(_sfm_data, _directory, _regionsType)){
    ROS_ERROR("Features provider load failed");
    return;
    }

  openMVG::PairWiseMatches map_GeometricMatches;

//no function collectionGeomFilter
  //ImageCollectionGeometricFilter collectionGeomFilter(feats_provider.get());
std::unique_ptr<ImageCollectionGeometricFilter> filter_ptr(new ImageCollectionGeometricFilter(&_sfm_data, regions_provider));

  const double maxResidualError = 1.0; // dflt 1.0; // Lower is stricter

//no member method "Filter" after library changes
//https://github.com/openMVG/openMVG/commit/fa8ab91c04dd0828ce60e9b7e74069f5cc54d4d6
/*
  collectionGeomFilter.Filter(
            GeometricFilter_FMatrix_AC(maxResidualError),
            map_PutativesMatches,
            map_GeometricMatches,
            vec_imagesSize);
*/
filter_ptr->Robust_model_estimation(GeometricFilter_FMatrix_AC(maxResidualError), map_PutativesMatches); //added
map_GeometricMatches = filter_ptr->Get_geometric_matches(); //added

  ofstream file_filtered(_matches_filtered); //changed file object name to avoid conflict
  if (file_filtered.is_open()){
    PairedIndMatchToStream(map_GeometricMatches, file_filtered);
    }
  file_filtered.close();

}

void ImageList::sequentialReconstruct() {
string output_directory = "/home/colin/catkin_ws/src/aslam_project/SfM_PointCloudData";
string sfm_data = stlplus::create_filespec(output_directory, "sfm_data", ".json");
string cloud_data = stlplus::create_filespec(output_directory, "cloud_and_poses", ".ply");
string report_name = stlplus::create_filespec(output_directory, "Reconstruction_Report", ".html");

  if (!stlplus::folder_exists(output_directory))
    stlplus::folder_create(output_directory);

  SequentialSfMReconstructionEngine sfmEngine(_sfm_data, output_directory, report_name);
  shared_ptr<Features_Provider> feats_provider = std::make_shared<Features_Provider>();
  shared_ptr<Matches_Provider> matches_provider = std::make_shared<Matches_Provider>();

  feats_provider->load(_sfm_data, _directory, _regionsType);
  matches_provider->load(_sfm_data, _matches_filtered);

  sfmEngine.SetFeaturesProvider(feats_provider.get());
  sfmEngine.SetMatchesProvider(matches_provider.get());

  openMVG::Pair initialPairIndex;
  Views::const_iterator it;

  it = _sfm_data.GetViews().begin();
  const View *v1 = it->second.get();
  it++;
  const View *v2 = it->second.get();

  initialPairIndex.first = v1->id_view;
  initialPairIndex.second = v2->id_view;

  sfmEngine.setInitialPair(initialPairIndex);
  sfmEngine.Set_bFixedIntrinsics(false);
  sfmEngine.SetUnknownCameraType(EINTRINSIC(PINHOLE_CAMERA_RADIAL3));

  sfmEngine.Process();
  Save(sfmEngine.Get_SfM_Data(), sfm_data, ESfM_Data(openMVG::sfm::ALL));
  Save(sfmEngine.Get_SfM_Data(), cloud_data, ESfM_Data(openMVG::sfm::ALL));
}

int LoadImageListing(ImageList* iml, const char* imagedir) {
  iml->setDirectory(imagedir);
  
  //ROS_ERROR("Image directory is: %s",imagedir);

  if (iml->countFiles() > 0) {
    //ROS_ERROR("Image count is: %i",iml->countFiles());
    iml->loadAllImages();
  }
return 0;
}


int main(int argc, char* argv[])
{
ImageList iml;

  if (argc != 2) {
    printf("supply a target directory\n");
    return 0;
  }
  LoadImageListing(&iml, argv[1]);
  iml.generateFeatures();
  iml.computeMatches();
  iml.sequentialReconstruct();
}
