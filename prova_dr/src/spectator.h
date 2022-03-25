#pragma once
#include "defs.h"
#include "environment.h"
#include "image.h"
#include "dtam.h"
#include "parameters.h"
#include "camera.h"

class Spectator{
  public:

    Spectator(Params* parameters, Dtam* dtam, colorRGB& background_color ):
    dtam_(dtam),
    background_color_(background_color),
    spectator_params_(initCamParams(parameters)),
    spectator_cam_(initCam(parameters)),
    spectator_image_(initImage(parameters))
    {};

    void spectateDso();
  private:
    Dtam* dtam_;
    colorRGB background_color_;
    CamParameters* spectator_params_;
    Camera* spectator_cam_;
    Image<colorRGB>* spectator_image_;

    CamParameters* initCamParams(Params* parameters);
    Camera* initCam(Params* parameters);
    Image<colorRGB>* initImage(Params* parameters);

    void renderState();
    void showSpectator();

    void reinitSpectator();
    void renderPoints();
    void renderCamsAndKFs();

    Eigen::Isometry3f getSpectatorPose();

    bool plotPt(ActivePoint* pt, const colorRGB& color);
    bool plotPt(Eigen::Vector3f& pt, const colorRGB& color);
    bool plotCam(CameraForMapping* cam, const colorRGB& color);
};
