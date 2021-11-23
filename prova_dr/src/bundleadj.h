#pragma once
#include "camera.h"
#include "image.h"
#include "environment.h"

class Dtam; //forward declaration

class BundleAdj{

  public:
    BundleAdj(Dtam* dtam): dtam_(dtam){};

    void projectAndMarginalizeActivePoints();
    void activateNewPoints();
    void optimize();
  private:
    Dtam* const dtam_;

};
