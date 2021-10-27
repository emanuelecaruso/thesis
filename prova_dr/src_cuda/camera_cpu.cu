#include "camera_cpu.cuh"
#include <thread>
#include <vector>
#include <mutex>

using namespace std;
using namespace pr;



void Camera_cpu::gpuFree(){
  image_rgb_gpu_.release();
  invdepth_map_gpu_.release();
}


void Camera_cpu::getCamera_gpu(){
  cudaError_t err ;

  image_rgb_gpu_.upload(image_rgb_->image_);
  invdepth_map_gpu_.upload(invdepth_map_->image_);

  int n_pixels=resolution_*(resolution_/aspect_);

  cudaMalloc(&cp_array_gpu_, sizeof(Cp_gpu)*n_pixels);
  err = cudaGetLastError();
  if (err != cudaSuccess)
      printf("cudaMalloc cps Error: %s\n", cudaGetErrorString(err));

  Camera_gpu* camera_gpu_h = new Camera_gpu(name_, lens_, aspect_, width_, resolution_,
     max_depth_,min_depth_, K_, Kinv_, *frame_camera_wrt_world_, *frame_world_wrt_camera_,
     *frame_camera_wrt_world_gt_, *frame_world_wrt_camera_gt_, invdepth_map_gpu_,
     image_rgb_gpu_, cp_array_gpu_);


  cudaMalloc((void**)&camera_gpu_, sizeof(Camera_gpu));
  err = cudaGetLastError();
  if (err != cudaSuccess)
      printf("cudaMalloc %s%s",name_," Error: %s\n", cudaGetErrorString(err));

  cudaMemcpy(camera_gpu_, camera_gpu_h, sizeof(Camera_gpu), cudaMemcpyHostToDevice);
  err = cudaGetLastError();
  if (err != cudaSuccess)
      printf("cudaMemcpy %s%s",name_," Error: %s\n", cudaGetErrorString(err));

  delete camera_gpu_h;
}

void Camera_cpu::cloneCameraImages(Camera* camera){
  invdepth_map_ = camera->invdepth_map_;
  image_rgb_ = camera->image_rgb_;

}

void Camera_cpu::showInvdepthmap(int scale){
  Image<float>* invdepthmap=new Image< float >("invdepth_"+name_);
  invdepth_map_gpu_.download(invdepthmap->image_);
  invdepthmap->show(scale/resolution_);
}

void Camera_cpu::setGroundtruthPose(){
  cudaFree(camera_gpu_);

  *frame_world_wrt_camera_=*frame_world_wrt_camera_gt_;
  *frame_camera_wrt_world_=*frame_camera_wrt_world_gt_;

  Camera_cpu::getCamera_gpu();

}
