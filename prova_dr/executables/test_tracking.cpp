#include "defs.h"
// #include "dataset.h"
#include "camera_cpu.cuh"
#include "image.h"
#include "environment.cuh"
#include "environment.h"
#include "renderer.cuh"
#include "utils.h"
#include "dtam_cuda.cuh"
#include <stdio.h>

using namespace std;
using namespace pr;


int main (int argc, char * argv[]) {


  //############################################################################
  // initialization
  //############################################################################
  std::string dataset_name = argv[1]; // dataset name
  std::string path_name = "./dataset/"+dataset_name;

  std::cout << "dataset name: " << dataset_name << std::endl;

  Environment_gpu* environment = new Environment_gpu(); // environment generator object (pointer)
  environment->loadEnvironment_gpu(path_name, dataset_name);

  Dtam* dtam = new Dtam(environment); // dense mapper and tracker


  //############################################################################
  // compute depth map
  //############################################################################

  dtam->test_tracking(environment);

  cv::waitKey(0);

  // --------------------------------------
  // show camera rgb images and depth maps
  // for (Camera_cpu* camera : environment->camera_vector_cpu_){
    // camera->image_rgb_->show(800/camera->resolution_);
    // camera->invdepth_map_->show(800/camera->resolution_);
  // }


  // // --------------------------------------
  return 1;
}
