#pragma once
#include "defs.h"
#include "camera.h"
#include "image.h"
#include "environment.h"

class Dtam; //forward declaration

class Tracker{

  public:
    Tracker(Dtam* dtam):
    dtam_(dtam){};

    void trackCam(bool takeGtPoses, bool track_candidates=false, int guess_type=VELOCITY_CONSTANT, bool debug_tracking=false);

    void collectCandidatesInCoarseRegions();
    void collectActivePointsInCoarseRegions();
    void filterOutOcclusionsGT();

  protected:
    Dtam* const dtam_;
    void trackGroundtruth();
    void trackLS(bool track_candidates=false, int guess_type=VELOCITY_CONSTANT, bool debug_tracking=false);
    Eigen::Isometry3f doLS(Eigen::Isometry3f& initial_guess, bool track_candidates=false, bool debug_tracking=false);

    void trackWithCandidates(Eigen::Isometry3f& current_guess, bool debug_tracking, CameraForMapping* frame_new);
    void trackWithActivePoints(Eigen::Isometry3f& current_guess, bool debug_tracking, CameraForMapping* frame_new);

    void collectCoarseCandidates(CameraForMapping* keyframe);

    void showProjectCandsWithCurrGuess( Eigen::Isometry3f& current_guess, int level);
    void showProjectActivePtsWithCurrGuess( Eigen::Isometry3f& current_guess, int level);

    bool iterationLSCands(Matrix6f& H, Vector6f& b, float& chi, Candidate* cand, CameraForMapping* frame_new, Eigen::Isometry3f& current_guess );
    bool iterationLS(Matrix6f& H, Vector6f& b, float& chi, ActivePoint* active_pt, CameraForMapping* frame_new, Eigen::Isometry3f& current_guess );
    bool updateLS(Matrix6f& H, Vector6f& b, float& chi, Eigen::Matrix<float, 2,6>& jacobian_to_mul, Eigen::Matrix<float, 2,1>& jacobian_to_mul_normalizer, pixelIntensity z, pixelIntensity z_hat, Eigen::Matrix<float, 1,2>& img_jacobian, float ni, float variance, float coeff, float invdepth_var );

    Eigen::Isometry3f computeInitialGuess( int guess_type=VELOCITY_CONSTANT );
    Eigen::Isometry3f computeInitialGuessGT( );
    Eigen::Isometry3f poseConstantModel();
    Eigen::Isometry3f velocityConstantModel();


};
