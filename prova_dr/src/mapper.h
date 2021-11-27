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
    Eigen::Vector2f cam_r_projected_in_cam_m;

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

      cam_m->projectCam(cam_r, cam_r_projected_in_cam_m);

      getSlopeParameters();
      getBoundsParameters();

    }
    EpipolarLine* getEpSegment(Candidate* candidate);
    EpipolarLine* getEpSegmentGt(Candidate* candidate);
    void compareEpSegmentWithGt(Candidate* candidate);
    void showEpSegment(Candidate* candidate);

    EpipolarLine* trackCandidate(Candidate* candidate);

  private:
    //Parameters for slope
    float A_s,B_s,C_s, D_s,E_s,F_s;
    //Parameters for bounds
    float A_bu,B_bu,C_bu,D_bu, E_bu,F_bu,G_bu,H_bu;
    float A_bv,B_bv,C_bv,D_bv, E_bv,F_bv,G_bv,H_bv;

    void getSlopeParameters();
    void getBoundsParameters();
    void getSlope(float u1, float v1, float& slope_m);
    void getBounds(float u1, float v1, float min_depth, float max_depth, float& bound_low, float& bound_up , bool u_or_v);
    void getBound(float u1, float v1, float d1, float& bound, bool u_or_v);

};

class Mapper{

  public:
    Mapper(Dtam* dtam, float threshold, int num_candidates):
      dtam_(dtam),
      threshold_(threshold),
      num_candidates_(num_candidates)
      {};

    void doMapping();
    void selectNewCandidates();
    void trackExistingCandidates();

  private:
    Dtam* const dtam_;
    const float threshold_;
    const int num_candidates_;
    int current_frame_;


    bool initializeCandidates(const CameraForMapping* cam_r,
                            const CameraForMapping* cam_m, int& current_r_idx);

};
