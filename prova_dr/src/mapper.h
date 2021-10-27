#pragma once
#include "camera.h"
#include "image.h"

class Dtam; //forward declaration

class Mapper{

  public:
    Mapper(Dtam* dtam): dtam_(dtam){};

  private:
    Dtam* const dtam_;

};
