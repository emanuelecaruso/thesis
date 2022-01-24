#pragma once
#include "camera.h"
#include "epline.h"

class Dtam; //forward declaration

class CamCouple{
  public:
    const CameraForMapping* cam_r_; // cam to be projected
    const CameraForMapping* cam_m_; // cam on which project
    Eigen::Isometry3f T; //cam_r expressed in cam_m
    Eigen::Matrix3f r;
    Eigen::Vector3f t;
    float f, f2, w, h, w2, h2;
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

      cam_m->projectCam(cam_r, cam_r_projected_in_cam_m);

      getSlopeParameters();
      getBoundsParameters();
      getDepthParameters();
    }
    EpipolarLine* getEpSegment(Candidate* candidate, int bound_idx);
    EpipolarLine* getEpSegmentDefaultBounds(float u1, float v1);
    // EpipolarLine* getEpSegmentGt(Candidate* candidate);
    // void compareEpSegmentWithGt(Candidate* candidate);
    // void showEpSegment(Candidate* candidate);

    void getBounds(float u1, float v1, float min_depth, float max_depth, float& bound_low, float& bound_up , bool u_or_v);
    void getCoord(float u1, float v1, float d1, float& coord, bool u_or_v);
    void getUv(float u1, float v1, float d1, float& u2, float& v2 );
    void getD1(float u1, float v1, float& d1, float coord, bool u_or_v);
    void getD2(float u1, float v1, float d1, float& d2);
    void getSlope(float u1, float v1, float& slope_m);

  private:
    //Parameters for slope
    float A_s,B_s,C_s, D_s,E_s,F_s;
    //Parameters for bounds
    float A_bu,B_bu,C_bu,D_bu, E_bu,F_bu,G_bu,H_bu;
    float A_bv,B_bv,C_bv,D_bv, E_bv,F_bv,G_bv,H_bv;
    //Parameters for depth
    float A_d, B_d, C_d, D_d;

    void getSlopeParameters();
    void getBoundsParameters();
    void getDepthParameters();




};

class Mapper{

  public:
    Mapper(Dtam* dtam, Params* parameters):
      dtam_(dtam),
      parameters_(parameters)
      {};

    void selectNewCandidates();

    void trackExistingCandidates(bool take_gt_points=false, bool debug_mapping=false);



  private:
    Dtam* const dtam_;
    Params* const parameters_;


    bool initializeCandidates(const CameraForMapping* cam_r,
                            const CameraForMapping* cam_m, int& current_r_idx);

    void updateBounds(Candidate* candidate, EpipolarLine* ep_line, CamCouple* cam_couple);

    CandidateProjected* projectCandidate(Candidate* candidate, CamCouple* cam_couple );
    CandidateProjected* projectCandidate(Candidate* candidate, CamCouple* cam_couple, EpipolarLine* ep_line );
    void trackExistingCandidates_( bool debug_mapping);
    void trackExistingCandidatesGT();
};
