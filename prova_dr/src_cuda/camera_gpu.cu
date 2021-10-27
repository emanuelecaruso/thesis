#include "camera_gpu.cuh"
#include <thread>
#include <vector>
#include <mutex>

using namespace std;
using namespace pr;



__device__ void Camera_gpu::pixelCoords2uv(Eigen::Vector2i& pixel_coords, Eigen::Vector2f& uv){
  float pixel_width = width_/resolution_;

  uv.x()=((float)pixel_coords.x()/(resolution_))*width_+(pixel_width/2);
  uv.y()=(((float)pixel_coords.y())/(float)((resolution_)/aspect_))*(float)(width_/aspect_)+(pixel_width/2);
}

__device__ void Camera_gpu::uv2pixelCoords( Eigen::Vector2f& uv, Eigen::Vector2i& pixel_coords){

  pixel_coords.x()=(int)((uv.x()/width_)*resolution_);
  pixel_coords.y()=(int)((uv.y()/(width_/aspect_))*(resolution_/aspect_));
}

__device__ void Camera_gpu::pointAtDepth(Eigen::Vector2f& uv, float depth, Eigen::Vector3f& p){

  Eigen::Vector3f p_proj;
  Eigen::Vector2f product = uv * depth;
  p_proj.x() = product.x();
  p_proj.y() = product.y();
  p_proj.z() = depth;
  Eigen::Vector3f p_cam = Kinv_*p_proj;
  p = frame_camera_wrt_world_*p_cam;

}

__device__ bool Camera_gpu::projectPoint(Eigen::Vector3f& p, Eigen::Vector2f& uv, float& p_cam_z ){

  Eigen::Vector3f p_cam = frame_world_wrt_camera_*p;

  // return wether the projected point is in front or behind the camera
  p_cam_z=-p_cam.z();

  Eigen::Vector3f p_proj = K_*p_cam;

  uv = p_proj.head<2>()*(1./p_proj.z());

  if (p_cam_z<lens_)
    return false;

  return true;

}

// __device__ bool Camera_gpu::projectPixel(Cp& cp){
//
//   Eigen::Vector2f uv;
//   float depth_cam;
//   bool point_in_front_of_camera = Camera_gpu::projectPoint(cp.point, uv, depth_cam );
//   if (!point_in_front_of_camera)
//     return false;
//
//   if(uv.x()<0 || uv.x()>width_)
//     return false;
//   if(uv.y()<0 || uv.y()>width_/aspect_)
//     return false;
//
//   Eigen::Vector2i pixel_coords;
//   Camera_gpu::uv2pixelCoords( uv, pixel_coords);
//
//   float depth = depth_cam/max_depth_;
//
//   float evaluated_pixel = invdepth_map_(pixel_coords.x(),pixel_coords.y());
//
//   if (evaluated_pixel<depth)
//     return false;
//
//   image_rgb_(pixel_coords.x(),pixel_coords.y()) = cp.color;
//   // image_rgb_->setPixel(pixel_coords, cp.color);
//   // invdepth_map_->setPixel(pixel_coords,depth);
//
//   return true;
// }

// __device__ void Camera_gpu::projectPixels(cpVector& cp_vector){
//   Camera_gpu::clearImgs();
//   for (Cp cp : cp_vector)
//   {
//     Camera_gpu::projectPixel(cp);
//   }
// }
