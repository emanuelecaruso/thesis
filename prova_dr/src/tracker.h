#pragma once
#include "camera.h"
#include "image.h"
#include "environment.h"

class Dtam; //forward declaration

class Tracker{

  public:
    Tracker(Dtam* dtam): dtam_(dtam){};

    void trackCam(bool takeGtPoses, bool const_acc=false);

  private:
    Dtam* const dtam_;

    void trackGroundtruth();
    void trackLS(bool const_acc);
    void doLS(Eigen::Isometry3f& initial_guess);
    Eigen::Isometry3f computeInitialGuess(bool const_acc );
    Eigen::Isometry3f velocityConstantModel();
    Eigen::Isometry3f accelerationConstantModel();
};
