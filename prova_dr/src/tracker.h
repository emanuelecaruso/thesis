#pragma once
#include "camera.h"
#include "image.h"
#include "environment.h"

class Dtam; //forward declaration

class Tracker{

  public:
    Tracker(Dtam* dtam): dtam_(dtam){};

    void trackCam(bool takeGtPoses, bool const_acc=true);

  private:
    Dtam* const dtam_;

    void trackGroundtruth();
    void computeInitialGuess(bool const_acc );
    Eigen::Isometry3f velocityConstantModel();
    Eigen::Isometry3f accelerationConstantModel();
};
