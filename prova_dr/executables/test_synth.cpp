#include "defs.h"
#include "dataset.h"
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

  int resolution = 600;
  float aspect = 1;
  int num_interpolations = 64;

  //############################################################################
  // initialization
  //############################################################################

  double t_start=getTime();  // time start for computing computation time
  double t_end=getTime();    // time end for computing computation time

  Environment* environment = new Environment(resolution, aspect); // environment generator object (pointer)
  Renderer* renderer = new Renderer(); // renderer object (pointer)
  Dtam* dtam = new Dtam(environment, num_interpolations); // dense mapper and tracker

  //############################################################################
  // generate 2 cameras (in this case same orientation, shift on x axis)
  //############################################################################

  // --------------------------------------
  // generate cameras
  float object_depth=2;
  float offset=-0.1;
  float offset_depth=-0.05;

  environment->generateCamera("camera_r", 0,0,-object_depth, 0,0,0);
  environment->generateCamera("camera_m1", offset,offset,-object_depth-offset_depth, 0,0,0);
  environment->generateCamera("camera_m2", -offset,offset,-object_depth-offset_depth, 0,0,0);
  environment->generateCamera("camera_m3", 0,offset,-object_depth-offset_depth, 0,0,0);
  environment->generateCamera("camera_m4", offset,-offset,-object_depth-offset_depth, 0,0,0);
  environment->generateCamera("camera_m5", -offset,-offset,-object_depth-offset_depth, 0,0,0);
  environment->generateCamera("camera_m6", 0,-offset,-object_depth-offset_depth, 0,0,0);
  environment->generateCamera("camera_m7", -offset,0,-object_depth-offset_depth, 0,0,0);
  environment->generateCamera("camera_m8", offset,0,-object_depth-offset_depth, 0,0,0);
  // --------------------------------------
  // generate environment
  cerr << "generating environment.." << endl;
  t_start=getTime();

  int density=5000;
  Eigen::Isometry3f pose_cube;
  // pose.linear().setIdentity();
  pose_cube.linear()=Rx(M_PI/6)*Ry(M_PI/4)*Rz(3.14/6);
  pose_cube.translation()= Eigen::Vector3f(0,0,-1);
  environment->generateTexturedCube(1, pose_cube, density);

  Eigen::Isometry3f pose_background;
  pose_background.linear().setIdentity();
  pose_background.translation()= Eigen::Vector3f(0,0,-1.8);
  environment->generateTexturedPlane("images/sunshine.jpg", 4, pose_background, density);

  // environment->generateSinusoidalSurface(object_depth, density);
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

  Camera* camera_r = environment->camera_vector_[0];
  Image<float>* depth_map_gt = camera_r->depth_map_->clone("depth map gt");
  // clear depth maps
  for (Camera* camera : environment->camera_vector_)
    camera->depth_map_->setAllPixels(1.0);



  //############################################################################
  // compute depth map
  //############################################################################

  int num_cameras = environment->camera_vector_.size();
  for (int it=0; it<num_cameras; it++){
    // --------------------------------------
    // for the first camera, set it as the reference camera
    if (it==0){
      dtam->addCamera(environment->camera_vector_[it]);
      dtam->setReferenceCamera(it);
      continue;
    }

    // --------------------------------------
    cerr << "computing discrete cost volume " << it << "/" << num_cameras-1 << endl;
    t_start=getTime();

    dtam->addCamera(environment->camera_vector_[it]);

    dtam->prepareCameraForDtam(it);

    // dtam->updateDepthMap(it, true);

    // dtam->updateDepthMap(it);
    dtam->updateDepthMap_parallel_cpu(it);


    t_end=getTime();
    cerr << "discrete cost volume computation took: " << (t_end-t_start) << " ms " << it << "/" << num_cameras-1 << endl;
    // --------------------------------------


  }
  // --------------------------------------
  // show camera rgb images and depth maps
  for (Camera* camera : environment->camera_vector_){
    camera->image_rgb_->show(800/camera->resolution_);
    // camera->depth_map_->show(800/camera->resolution_);
  }
  environment->camera_vector_[0]->depth_map_->show(800/environment->camera_vector_[0]->resolution_);
  depth_map_gt->show(800/environment->camera_vector_[0]->resolution_);
  cv::waitKey(0);
  // --------------------------------------
  return 1;
}
