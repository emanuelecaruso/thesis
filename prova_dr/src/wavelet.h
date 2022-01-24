#pragma once
#include "defs.h"
#include "image.h"

class CameraForMapping; // forward declaration
class Wvlt_dec; // forward declaration

class Wvlt_lvl{
  public:
    const int level;
    const Wvlt_dec* wvlt_dec;

    Image<pixelIntensity>* c;
    Image<pixelIntensity>* c_dx;
    Image<pixelIntensity>* c_dy;
    Image<pixelIntensity>* c_d2x;
    Image<pixelIntensity>* c_d2y;
    Image<float>* magn_cd;
    Image<float>* magn_cd_dx;
    Image<float>* magn_cd_dy;
    Image<float>* phase_cd;
    Image<float>* phase_cd_dx;
    Image<float>* phase_cd_dy;

    // clone
    Wvlt_lvl(Image<pixelIntensity>* c_,
            Image<pixelIntensity>* c_dx_, Image<pixelIntensity>* c_dy_,
            Image<pixelIntensity>* c_d2x_, Image<pixelIntensity>* c_d2y_,
            Image<float>* magn_cd_,
            Image<pixelIntensity>* magn_cd_dx, Image<pixelIntensity>* magn_cd_dy,
            Image<float>* phase_cd_,
            Image<pixelIntensity>* phase_cd_dx, Image<pixelIntensity>* phase_cd_dy,
           const int level_, const Wvlt_dec* wvlt_dec_):
              level(level_),
              wvlt_dec(wvlt_dec_),
              c(c_->clone()),
              c_dx(c_dx_->clone()),
              c_dy(c_dy_->clone()),
              c_d2x(c_d2x_->clone()),
              c_d2y(c_d2y_->clone()),
              magn_cd(magn_cd_->clone()),
              magn_cd_dx(magn_cd_dx->clone()),
              magn_cd_dy(magn_cd_dy->clone()),
              phase_cd(phase_cd->clone()),
              phase_cd_dx(phase_cd_dx->clone()),
              phase_cd_dy(phase_cd_dy->clone())
    {  };

    // create first level
    Wvlt_lvl(const Image<pixelIntensity>* img, Wvlt_dec* wvlt_dec_ ):
    level(0),
    wvlt_dec(wvlt_dec_){
      WaveletDecHaar(img);
    };

    // create next level
    Wvlt_lvl(Wvlt_lvl* wvlt_lvl_previous):
    level(wvlt_lvl_previous->level+1),
    wvlt_dec(wvlt_lvl_previous->wvlt_dec){
      // WaveletDecHaar( wvlt_lvl_previous);
      WaveletDecHaar( wvlt_lvl_previous->c);
    };
    inline Wvlt_lvl* clone(){
      return new Wvlt_lvl(c,c_dx,c_dy,c_d2x,c_d2y, magn_cd, magn_cd_dx, magn_cd_dy,phase_cd,phase_cd_dx, phase_cd_dy, level, wvlt_dec);};

  private:
    void WaveletDecHaar(const Image<pixelIntensity>* img);
    Image<float>* getMagnitude(Image<pixelIntensity>* dx, Image<pixelIntensity>* dy);
    Image<float>* getPhase(Image<pixelIntensity>* dx, Image<pixelIntensity>* dy);
    // void WaveletDecHaar(Wvlt_lvl* wvlt_lvl_previous);

};



class Wvlt_dec{

  public:
    const int levels_;

    const CameraForMapping* cam_;
    const Image<pixelIntensity>* image_;
    std::vector< Wvlt_lvl* >* vector_wavelets;

    // clone wlt decomposition
    Wvlt_dec( Wvlt_dec* wvlt_dec ):
    levels_( wvlt_dec->levels_ ),
    image_( wvlt_dec->image_ ),
    cam_( wvlt_dec->cam_ ),
    vector_wavelets(new std::vector<Wvlt_lvl*>(levels_))
    {
      for (int i=0; i<levels_; i++){
        Wvlt_lvl* wvlt_lvl = wvlt_dec->vector_wavelets->at(i);
        vector_wavelets->at(i)=wvlt_lvl->clone();
      }

    }

    // compute wlt decomposition
    Wvlt_dec(int levels,const Image<pixelIntensity>* img,const CameraForMapping* cam ):
    levels_(levels),
    image_(img),
    cam_(cam),
    vector_wavelets(new std::vector<Wvlt_lvl*>(levels_))
    {
      Wvlt_lvl* wvlt_0 = new Wvlt_lvl( image_, this );
      vector_wavelets->at(0) = wvlt_0;

      for(int i=1; i<levels_; i++){
        Wvlt_lvl* wvlt_i = new Wvlt_lvl( vector_wavelets->at(i-1) );
        // Wvlt_lvl* wvlt_i = new Wvlt_lvl( vector_wavelets->at(i-1)->c, this );
        vector_wavelets->at(i)=wvlt_i;
      }
    }


    // void signThresholdedPoints(float threshold, bool printNPix=false);
    // void compareThreshold(float threshold, float size=1);
    Wvlt_lvl* getWavLevel(int level);
    void showWaveletDec(float size=1);
    void showWaveletDec(const std::string& name, float size=1);

};
