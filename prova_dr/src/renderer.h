#pragma once
#include "defs.h"
#include "environment.h"
#include "image.h"
#include "camera.h"

using namespace pr;

class Renderer{
  public:

    bool renderPoint(Cp& cp, Camera* camera);
    bool renderImage_naive(cpVector& cp_vector, Camera* camera);
    bool renderImages_parallel_cpu(Environment* environment);
};
