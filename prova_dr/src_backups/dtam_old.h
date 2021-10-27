#pragma once
#include "camera.h"
#include "image.h"

class Dtam{
  public:
    int i_;
    Image<float>* img_cam_r_;
    Image<float>* img_cam_m_;
    Dtam(int i){
      i_ = i;
    };

    bool get1stDepthWithUV(Camera* camera_r, Camera* camera_m, Eigen::Vector2f& uv_r, Eigen::Vector2f& uv_m, float& depth);
    bool collectCostsInEpipolarLine_FVT(Eigen::Vector2i& uv_r, Camera* camera_r, Camera* camera_m, std::vector<std::tuple<float,int>>& costInvdepthTuple);
    bool collectCostsInEpipolarLine(Eigen::Vector2i& uv_r, Camera* camera_r, Camera* camera_m, std::vector<std::tuple<float,int>>& costInvdepthTuple);
    bool getTotalCosts(std::vector<std::vector<std::tuple<float,int>>>& allCosts, std::vector<std::tuple<float,int>>& totalCosts);
    int getIndexOfMinimumCost(std::vector<std::tuple<float,int>>& totalCosts);
    void getDepthMap(CameraVector& camera_vector);
    int closest(std::vector<float>& vec, float value);

  private:
    // taken from https://stackoverflow.com/questions/8647635/elegant-way-to-find-closest-value-in-a-vector-from-above
    void extractDepthsFromTupleVec(std::vector<std::tuple<float,int>>& totalCosts, std::vector<float>& invdepths);
    void extractCostsFromTupleVec(std::vector<std::tuple<float,int>>& totalCosts, std::vector<float>& costs);

};
