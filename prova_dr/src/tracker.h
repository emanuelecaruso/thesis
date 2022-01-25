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

    void trackCam(bool takeGtPoses, bool track_candidates=false, int guess_type=VELOCITY_CONSTANT);

    void collectCandidatesInCoarseRegions();

  protected:
    Dtam* const dtam_;
    void trackGroundtruth();
    void trackLS(bool track_candidates=false, int guess_type=VELOCITY_CONSTANT);
    Eigen::Isometry3f doLS(Eigen::Isometry3f& initial_guess, bool track_candidates=false);
    void collectCoarseCandidates(CameraForMapping* keyframe);
    void showProjectCandsWithCurrGuess( Eigen::Isometry3f& current_guess, int level);
    bool iterationLS(Matrix6f& H, Vector6f& b, float& chi, Candidate* cand, CameraForMapping* frame_new, Eigen::Isometry3f& current_guess );
    Eigen::Isometry3f computeInitialGuess( int guess_type=VELOCITY_CONSTANT );
    Eigen::Isometry3f computeInitialGuessGT( );
    Eigen::Isometry3f poseConstantModel();
    Eigen::Isometry3f velocityConstantModel();
    void filterOutOcclusionsGT();


};
