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
    Image<pixelIntensity>* dh;
    Image<pixelIntensity>* dv;
    Image<pixelIntensity>* dh_robust;
    Image<pixelIntensity>* dv_robust;
    Image<float>* magnitude_img;

    // clone
    Wvlt_lvl(Image<pixelIntensity>* c_,
            Image<pixelIntensity>* dh_, Image<pixelIntensity>* dv_,
            Image<pixelIntensity>* dh_robust_, Image<pixelIntensity>* dv_robust_,
            Image<float>* magnitude_img_,
           const int level_, const Wvlt_dec* wvlt_dec_):
              level(level_),
              wvlt_dec(wvlt_dec_),
              c(c_->clone()),
              dh(dh_->clone()),
              dv(dv_->clone()),
              dh_robust(dh_robust_->clone()),
              dv_robust(dv_robust_->clone()),
              magnitude_img(magnitude_img_->clone())
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
      return new Wvlt_lvl(c,dh,dv,dh_robust,dv_robust, magnitude_img, level, wvlt_dec);};

    Image<float>* compute_robust_sobels();

  private:
    void WaveletDecHaar(const Image<pixelIntensity>* img);
    Image<float>* getMagnitude();
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
