#pragma once
#include "camera.h"
#include "image.h"

class Dtam{
  public:
    int i_;
    Dtam(int i){
      i_ = i;
    };

    bool get1stDepthWithUV(Camera* camera_r, Camera* camera_m, Eigen::Vector2f& uv_r, Eigen::Vector2f& uv_m, float& depth);
    void getDepthMap(CameraVector& camera_vector, float interpolation_ratio, bool check=false);

};
