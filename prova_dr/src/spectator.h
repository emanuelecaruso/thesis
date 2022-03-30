#pragma once
#include "defs.h"
#include "environment.h"
#include "image.h"
#include "parameters.h"
#include "camera.h"

class Dtam;

class Spectator{
  public:

    Spectator(Dtam* dtam, Params* parameters, const colorRGB& background_color ):
    dtam_(dtam),
    background_color_(background_color),
    spectator_params_(initCamParams(parameters)),
    spectator_cam_(initCam(parameters)),
    spectator_image_(initImage(parameters))
    {};

    void spectateDso();
    void renderState();
    void showSpectator();
    
  private:
    Dtam* dtam_;
    colorRGB background_color_;
    CamParameters* spectator_params_;
    Camera* spectator_cam_;
    Image<colorRGB>* spectator_image_;

    CamParameters* initCamParams(Params* parameters);
    Camera* initCam(Params* parameters);
    Image<colorRGB>* initImage(Params* parameters);


    void reinitSpectator();
    void renderPoints();
    void renderCamsAndKFs();

    Eigen::Isometry3f getSpectatorPose();

    bool plotPt(ActivePoint* pt, const colorRGB& color);
    bool plotPt(Eigen::Vector3f& pt, const colorRGB& color);
    bool plotPt(ActivePoint* pt, const colorRGB& color, pxl& pixel);
    bool plotPt(Eigen::Vector3f& pt, const colorRGB& color, pxl& pixel);
    bool plotLine(pxl& pixel1, pxl& pixel2, const colorRGB& color );
    bool plotCam(Camera* cam, const colorRGB& color);
};
