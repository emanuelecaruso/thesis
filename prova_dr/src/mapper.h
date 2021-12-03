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
    cam_m_(cam_m),
    T((*(cam_m->frame_world_wrt_camera_))*(*(cam_r->frame_camera_wrt_world_))),
    f(cam_r->cam_parameters_->lens), f2(f*f), w(cam_r->cam_parameters_->width),
    w2(w*w), h(cam_r->cam_parameters_->height), h2(h*h)
    {
      r=T.linear();
      t=T.translation();
      t22=t(2)*t(2);

      cam_m->projectCam(cam_r, cam_r_projected_in_cam_m);

      getSlopeParameters();
      getBoundsParameters();

    }
    EpipolarLine* getEpSegment(Candidate* candidat, int bound_idx);
    // EpipolarLine* getEpSegmentGt(Candidate* candidate);
    // void compareEpSegmentWithGt(Candidate* candidate);
    // void showEpSegment(Candidate* candidate);

    EpipolarLine* trackCandidate(Candidate* candidate, int bound_idx);

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
    void getDepth(float u1, float v1, float& d1, float u2, bool u_or_v);



};

class Mapper{

  public:
    Mapper(Dtam* dtam, float grad_threshold, float cost_threshold, int num_candidates):
      dtam_(dtam),
      grad_threshold_(grad_threshold),
      cost_threshold_(cost_threshold),
      num_candidates_(num_candidates),
      current_frame_(0)
      {};

    void doMapping();
    void selectNewCandidates();
    void trackExistingCandidates();

  private:
    Dtam* const dtam_;
    const float grad_threshold_;
    const float cost_threshold_;
    const int num_candidates_;
    int current_frame_;


    bool initializeCandidates(const CameraForMapping* cam_r,
                            const CameraForMapping* cam_m, int& current_r_idx);

    void updateBounds(Candidate* candidate, EpipolarLine* ep_line);

};
