#include "renderer.h"
#include <thread>
#include <vector>
#include <mutex>

bool Renderer::renderPoint(Cp& cp, Camera* camera){

  Eigen::Vector2f uv;
  float depth_cam;
  bool point_in_front_of_camera = camera->projectPoint(cp.point, uv, depth_cam );
  if (!point_in_front_of_camera)
    return false;

  float width = camera->width_;
  float height = camera->width_/camera->aspect_;

  if(uv.x()<0 || uv.x()>width)
    return false;
  if(uv.y()<0 || uv.y()>height)
    return false;

  Eigen::Vector2i pixel_coords;
  camera->uv2pixelCoords( uv, pixel_coords);

  float depth = depth_cam/camera->max_depth_;

  float evaluated_pixel;
  camera->invdepth_map_->evalPixel(pixel_coords,evaluated_pixel);

  if (evaluated_pixel<depth)
    return false;

  if (depth>1 || depth>255 || cp.color[0]>255 || cp.color[1]>255 || cp.color[2]>255)
    return false;

  cv::Vec3b color = cv::Vec3b(cp.color[0],cp.color[1],cp.color[2]);

  camera->image_rgb_->setPixel(pixel_coords, color);
  camera->invdepth_map_->setPixel(pixel_coords,depth);

  return true;
}

bool Renderer::renderImage_naive(cpVector& cp_vector, Camera* camera){

    camera->clearImgs();
    for (Cp cp : cp_vector)
    {
      Renderer::renderPoint(cp, camera);
    }

  return true;
}

bool Renderer::renderImages_parallel_cpu(Environment* environment){

  // camera->clearImgs();
  const size_t nloop = environment->cp_vector_.size();
  const size_t nthreads = std::thread::hardware_concurrency();
  {
    // Pre loop
    std::vector<std::thread> threads(nthreads);
    std::mutex critical;
    for(int t = 0;t<nthreads;t++)
    {
      threads[t] = std::thread(std::bind(
        [&](const int bi, const int ei, const int t)
        {
          // loop over all items
          for(int i = bi;i<ei;i++)
          {
            // inner loop
            {
              for (int j=0; j<environment->camera_vector_.size(); j++){
                Renderer::renderPoint(environment->cp_vector_[i],
                   environment->camera_vector_[j]);
              }

            }
          }
        },t*nloop/nthreads,(t+1)==nthreads?nloop:(t+1)*nloop/nthreads,t));
    }
    std::for_each(threads.begin(),threads.end(),[](std::thread& x){x.join();});
    // Post loop
  }

  return true;
}
