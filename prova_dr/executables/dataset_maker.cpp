#include "defs.h"
// #include "dataset.h"
#include "camera.h"
#include "image.h"
#include "environment.h"
#include "renderer.h"
#include "utils.h"
#include "dtam.h"
#include <stdio.h>
#include <cuda_runtime.h>


using namespace std;
using namespace pr;


int main (int argc, char * argv[]) {


  //############################################################################
  // choose parameters
  //############################################################################

  int resolution = 64;
  float aspect = 1.333333333;
  // float aspect = 1;

  //############################################################################
  // initialization
  //############################################################################

  double t_start=getTime();  // time start for computing computation time
  double t_end=getTime();    // time end for computing computation time

  Environment* environment = new Environment(resolution, aspect); // environment generator object (pointer)
  Renderer* renderer = new Renderer(); // renderer object (pointer)

  //############################################################################
  // generate 2 cameras (in this case same orientation, shift on x axis)
  //############################################################################

  // --------------------------------------

  // std::string dataset_name = "rotatedcube_9cams_64res"; // dataset name

  // std::string dataset_name = "rotatedcube_2cams"; // dataset name
  // std::string dataset_name = "rotatedcube_9cams"; // dataset name
  // std::string dataset_name = "rotatedcube_17cams"; // dataset name
  // std::string dataset_name = "rotatedcube_25cams"; // dataset name
  std::string dataset_name = "sin_9cams_64res"; // dataset name
  std::string path_name = "./dataset/"+dataset_name; // dataset name

  // generate cameras
  float object_depth=2;
  float offset=-0.1;
  float offset_depth=-0.05;

  environment->generateCamera("camera00", 0,0,-object_depth, 0,0,0);
  environment->generateCamera("camera01", offset,offset,-object_depth-offset_depth, 0,0,0);
  environment->generateCamera("camera02", -offset,offset,-object_depth-offset_depth, 0,0,0);
  environment->generateCamera("camera03", 0,offset,-object_depth-offset_depth, 0,0,0);
  environment->generateCamera("camera04", offset,-offset,-object_depth-offset_depth, 0,0,0);
  environment->generateCamera("camera05", -offset,-offset,-object_depth-offset_depth, 0,0,0);
  environment->generateCamera("camera06", 0,-offset,-object_depth-offset_depth, 0,0,0);
  environment->generateCamera("camera07", -offset,0,-object_depth-offset_depth, 0,0,0);
  environment->generateCamera("camera08", offset,0,-object_depth-offset_depth, 0,0,0);

  // environment->generateCamera("camera09", offset/2,offset/2,-object_depth-offset_depth, 0,0,0);
  // environment->generateCamera("camera10", -offset/2,offset/2,-object_depth-offset_depth, 0,0,0);
  // environment->generateCamera("camera11", 0,offset/2,-object_depth-offset_depth, 0,0,0);
  // environment->generateCamera("camera12", offset/2,-offset/2,-object_depth-offset_depth, 0,0,0);
  // environment->generateCamera("camera13", -offset/2,-offset/2,-object_depth-offset_depth, 0,0,0);
  // environment->generateCamera("camera14", 0,-offset/2,-object_depth-offset_depth, 0,0,0);
  // environment->generateCamera("camera15", -offset/2,0,-object_depth-offset_depth, 0,0,0);
  // environment->generateCamera("camera16", offset/2,0,-object_depth-offset_depth, 0,0,0);
  //
  // environment->generateCamera("camera17", offset*1.5,offset*1.5,-object_depth-offset_depth, 0,0,0);
  // environment->generateCamera("camera18", -offset*1.5,offset*1.5,-object_depth-offset_depth, 0,0,0);
  // environment->generateCamera("camera19", 0,offset*1.5,-object_depth-offset_depth, 0,0,0);
  // environment->generateCamera("camera20", offset*1.5,-offset*1.5,-object_depth-offset_depth, 0,0,0);
  // environment->generateCamera("camera21", -offset*1.5,-offset*1.5,-object_depth-offset_depth, 0,0,0);
  // environment->generateCamera("camera22", 0,-offset*1.5,-object_depth-offset_depth, 0,0,0);
  // environment->generateCamera("camera23", -offset*1.5,0,-object_depth-offset_depth, 0,0,0);
  // environment->generateCamera("camera24", offset*1.5,0,-object_depth-offset_depth, 0,0,0);


  // --------------------------------------
  // generate environment
  cerr << "generating environment.." << endl;
  t_start=getTime();

  // int density=5000;
  // Eigen::Isometry3f pose_cube;
  // // pose.linear().setIdentity();
  // pose_cube.linear()=Rx(M_PI/6)*Ry(M_PI/4)*Rz(3.14/6);
  // pose_cube.translation()= Eigen::Vector3f(0,0,-1);
  // environment->generateTexturedCube(1, pose_cube, density);
  //
  // Eigen::Isometry3f pose_background;
  // pose_background.linear().setIdentity();
  // pose_background.translation()= Eigen::Vector3f(0,0,-1.8);
  // environment->generateTexturedPlane("images/sunshine.jpg", 4, pose_background, density);

  int density=5000;
  environment->generateSinusoidalSurface(object_depth, density);
  // environment->generateTexturedPlane("images/leon.jpg", 1, pose, density);

  t_end=getTime();
  cerr << "environment generation took: " << (t_end-t_start) << " ms" << endl;
  // --------------------------------------

  //############################################################################
  // generate depth map groundtruth and rgb images of cameras
  //############################################################################

  // --------------------------------------
  // rendering environment on cameras
  cerr << "rendering environment on cameras..." << endl;
  t_start=getTime();

  renderer->renderImages_parallel_cpu(environment);

  t_end=getTime();
  cerr << "rendering took: " << (t_end-t_start) << " ms" << endl;
  // --------------------------------------


  environment->saveEnvironment( path_name, dataset_name);

  // std::cout << "PRINT BEFORE" << std::endl;
  // for (Camera* camera : environment->camera_vector_){
  //   camera->printMembers();
  //   camera->image_rgb_->show();
  //   camera->depth_map_->show();
  // }
  //
  // // Check saved env
  // environment->loadEnvironment( path_name, dataset_name);
  //
  // std::cout << "PRINT AFTER" << std::endl;
  // for (Camera* camera : environment->camera_vector_){
  //   camera->printMembers();
  //   camera->image_rgb_->show(1, "_after");
  //   camera->depth_map_->show(1, "_after");
  // }
  cv::waitKey(0);

  return 1;
}
