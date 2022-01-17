#pragma once
#include "camera.h"
#include "image.h"
#include "environment.h"

class Dtam; //forward declaration

class Tracker{

  public:
    Tracker(Dtam* dtam):
    dtam_(dtam){};

    void trackCam(bool takeGtPoses, bool track_candidates=false);

  private:
    Dtam* const dtam_;
    void trackGroundtruth();
    void trackLS(bool track_candidates=false);
    Eigen::Isometry3f doLS(Eigen::Isometry3f& initial_guess, bool track_candidates=false);
    Eigen::Isometry3f computeInitialGuess( );
    Eigen::Isometry3f velocityConstantModel();
    Eigen::Isometry3f accelerationConstantModel();
};
