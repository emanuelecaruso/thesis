#pragma once
#include "camera.h"
#include "image.h"
#include "environment.h"

class Dtam; //forward declaration

class Tracker{

  public:
    Tracker(Dtam* dtam): dtam_(dtam){};

  private:
    Dtam* const dtam_;
};
