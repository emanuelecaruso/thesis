#include "environment.h"
#include "defs.h"
#include "json.hpp"
#include "utils.h"
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
using json = nlohmann::json;


std::vector<Camera*>* Environment::loadCameraVector(const std::string& path_name, const std::string& dataset_name, int end_frame){

  std::vector<Camera*>* camera_vector = nullptr;

  const char* path_name_ = path_name.c_str(); // dataset name

  std::string complete_path = path_name+"/"+dataset_name+".json";

  struct stat info;
  if( stat( path_name_, &info ) != 0 )
    return camera_vector;
  else if( info.st_mode & S_IFDIR )
    {}
  else
    return camera_vector;



  // read a JSON file
  std::ifstream i(complete_path);
  json j;
  i >> j;


  auto cameras = j.at("cameras");

  camera_vector = new std::vector<Camera*>;

  int count =0;

  for (json::iterator it = cameras.begin(); it != cameras.end(); ++it) {

    if (count>end_frame)
      break;

    std::string name=it.key();


    struct stat info_;
    std::string path_rgb_=(path_name+"/rgb_"+name+".png");
    const char* path_rgb = path_rgb_.c_str(); // dataset name
    if( stat( path_rgb, &info_ ) != 0 )
      continue;

    nlohmann::basic_json<>::value_type f;
    int frame;
    try{
      frame= it.value().at("frame");
      f= it.value().at("pose");
    } catch (std::exception& e) {
      std::string error = ": missing values in json file for cameras";
      std::cout << error << std::endl;
      return camera_vector;
    };


    struct stat info__;
    std::string path_depth_=(path_name+"/depth_"+name+".exr");
    const char* path_depth = path_depth_.c_str(); // dataset name
    if( stat( path_depth, &info__ ) != 0 ){
      Camera* camera = new Camera(name,cam_parameters_, f, path_rgb_ );
      camera_vector->push_back(camera);
      std::cout << camera->name_ << " added in env" << std::endl;
      count++;
    }
    else{
      Camera* camera = new Camera(name,cam_parameters_, f, path_rgb_, path_depth_);
      camera_vector->push_back(camera);
      std::cout << camera->name_ << " added in env" << std::endl;
      count++;
    }

  }
  return camera_vector;

}


CamParameters* Environment::loadCamParameters(const std::string& path_name, const std::string& dataset_name){

  CamParameters* cam_parameters_out = nullptr;

  const char* path_name_ = path_name.c_str(); // dataset name

  std::string complete_path = path_name+"/"+dataset_name+".json";

  struct stat info;
  if( stat( path_name_, &info ) != 0 )
  {
    printf( "ERROR: Dataset NOT found: %s \n", path_name_ );
    exit(1);
    // return cam_parameters_out;
  }
  else if( info.st_mode & S_IFDIR )
  {
    printf( "Dataset found: %s \n",path_name_ );
  }
  else
  {
    printf( "ERROR: %s Is not a directory\n", path_name_ );
    exit(1);
    // return cam_parameters_out;
  }



  // read a JSON file
  std::ifstream i(complete_path);
  json j;
  i >> j;


  auto environment = j.at("environment");
  try{
    const float lens = environment.at("lens");
    const float max_depth = environment.at("max_depth");
    const float min_depth = environment.at("min_depth");
    const float width = environment.at("width");
    const int resolution_x = environment.at("resolution_x");
    const int resolution_y = environment.at("resolution_y");

    cam_parameters_out = new CamParameters(
      resolution_x, resolution_y, width,
      lens, min_depth, max_depth);

  } catch (std::exception& e) {
    std::string error = ": missing values in json file for environment";
    std::cout << error << std::endl;
    return cam_parameters_out;
  };

  return cam_parameters_out;

}

void Environment::debugAllCameras(bool show_imgs) const {

  std::cout << "DEBUGGING ALL CAMERAS:" << std::endl;
  std::cout << "camera vector size: " << camera_vector_->size() << std::endl;

  for(Camera* camera : *camera_vector_){
    camera->printMembers();
    if (show_imgs){
      camera->showRGB();
      camera->showDepthMap();
    }
  }
  cv::waitKey(0);
}
