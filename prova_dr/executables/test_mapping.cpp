#include "environment.h"
#include "dtam.h"
#include <stdio.h>

using namespace std;
using namespace pr;


int main (int argc, char * argv[]) {


  // read arguments
  const std::string dataset_name = argv[1]; // dataset name
  const std::string path_name = "./dataset/"+dataset_name;

  // initialization
  Environment* environment = new Environment(path_name, dataset_name); // environment generator object (pointer)
  Dtam* dtam = new Dtam(environment); // dense mapper and tracker

  //############################################################################
  // compute depth map
  //############################################################################

  dtam->test_mapping();
  // dtam->testFeatures();
  // cv::waitKey(0);

  // --------------------------------------
  // show camera rgb images and depth maps
  // for (Camera_cpu* camera : environment->camera_vector_cpu_){
    // camera->image_rgb_->show(800/camera->resolution_);
    // camera->invdepth_map_->show(800/camera->resolution_);
  // }


  // // --------------------------------------
  return 1;
}
