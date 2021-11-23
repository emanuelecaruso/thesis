#pragma once
#include "camera.h"
#include "epline.h"

class Dtam; //forward declaration

class CamCouple{
  public:
    const CameraForMapping* cam_r_;
    const CameraForMapping* cam_m_;
    Eigen::Isometry3f T; //cam_r expressed in cam_m
    Eigen::Matrix3f r;
    Eigen::Vector3f t;
    float f, f2, w, h, w2, h2, t22;

    CamCouple(const CameraForMapping* cam_r, const CameraForMapping* cam_m):
    cam_r_(cam_r),
    cam_m_(cam_m)
    {
      T=(*(cam_m->frame_world_wrt_camera_))*(*(cam_r->frame_camera_wrt_world_));
      r=T.linear();
      t=T.translation();

      f = cam_r->cam_parameters_->lens;
      f2 = f*f;
      w = cam_r->cam_parameters_->width;
      h = cam_r->cam_parameters_->height;
      w2 = w*w;
      h2 = h*h;
      t22=t(2)*t(2);

      getSlopeParameters();
      getBoundsParameters();

    }
    EpipolarLine* getEpSegment(float u1, float v1);
    EpipolarLine* compareEpSegmentWithGt(float u1, float v1);

  private:
    //Parameters for slope
    float A_s,B_s,C_s, D_s,E_s,F_s;
    //Parameters for bounds
    float A_bu,B_bu,C_bu,D_bu, E_bu,F_bu,G_bu,H_bu;
    float A_bv,B_bv,C_bv,D_bv, E_bv,F_bv,G_bv,H_bv;

    void getSlopeParameters();
    void getBoundsParameters();
    void getSlope(float u1, float v1, float& slope_m);
    void getBounds(float u1, float v1, float d1, float& bound_low, float& bound_up);

};

class Mapper{

  public:
    Mapper(Dtam* dtam, float threshold, int num_candidates):
      dtam_(dtam),
      threshold_(threshold),
      num_candidates_(num_candidates)
      {};

    void doMapping();
    void selectCandidates();
    void propagateOldCandidates();

  private:
    Dtam* const dtam_;
    const float threshold_;
    const int num_candidates_;
    int current_frame_;

    // frame coupling
    void frameCouplingRandom(int& frame_1, int& frame_2);
    void frameCouplingLast(int& frame_1, int& frame_2);
    void frameCouplingOpposite(int& frame_1, int& frame_2);

    bool computeEpipolarLineCouple(const CameraForMapping* cam_1, const CameraForMapping* cam_2,
                                Eigen::Vector2f& uv_1, EpipolarLine*& ep_line_1,
                                EpipolarLine*& ep_line_2);

    float coord2FromCoord1(float coord1, Eigen::Vector4f& abcd);

    void getParametersABCD( EpipolarLine* ep_line_source, EpipolarLine* ep_line_range,
                                float depth, Eigen::Vector4f& abcd);

    void showRangeStudy(EpipolarLine* ep_line_source, EpipolarLine* ep_line_range,
                            int uvs_idx, float size=1);

    bool initializeCandidates(const CameraForMapping* cam_r,
                            const CameraForMapping* cam_m, int& current_r_idx);

};
