#pragma once
#include "camera.h"
#include "image.h"

class Dtam; //forward declaration




class EpipolarLine{
  public:

    // line parameters

    // defines wether start, end and x0 are expressed as  u or v coordinate, true->u, false->v
    const float slope;  // slope of the line
    const bool u_or_v;
    const float c0;     // v at u=0, or u at v=0

    // slope is kept const, start and end can be moved
    float start;  // start of segment (u or v coord)
    float end;    // end of segment (u or v coord)

    const CameraForMapping* cam; // camera associated to epipolar line
    std::vector<Eigen::Vector2f>* uvs;  // vector of interpoled uvs along epipolar line
    std::vector<int>* uv_idxs_mins;  // idx of the min costs along epipolar line

    EpipolarLine( const CameraForMapping* cam_, float slope_, float c_start_, float c_end_, float c0_, int level=-1 ):
    slope( slope_ ),
    u_or_v( (slope_<1 && slope_>-1) ),
    c0( c0_ ),
    cam(cam_),
    start(c_start_), end(c_end_),
    uvs(new std::vector<Eigen::Vector2f>),
    uv_idxs_mins(new std::vector<int>)
    {
      lineTraverse(level);
    }

    EpipolarLine( const CameraForMapping* cam_, float slope_, float c_start_,
                  float c_end_, Eigen::Vector2f& cam_proj_ , int level=-1):
    EpipolarLine( cam_, slope_, c_start_, c_end_, computeC0( cam_proj_, slope_), level  ){}


    EpipolarLine( const CameraForMapping* cam_, Eigen::Vector2f& start_, Eigen::Vector2f& end_, int level=-1):
    slope( (end_-start_).y()/(end_-start_).x() ),
    u_or_v( (slope<1 && slope>-1) ),
    c0( u_or_v ? start_.y()-slope*start_.x() : start_.x()-start_.y()/slope ),
    cam(cam_)
    {
      UVToCoord(start_,start);
      UVToCoord(end_,end);
      lineTraverse(level);

    }

    void printMembers() const;

    // show
    void showEpipolar(int level=-1, float size=1);
    void showEpipolarWithMin(int level=-1, float size=1);
    void showEpipolarComparison(EpipolarLine* ep_line_2, bool print, float size);
    void showEpipolarComparison(EpipolarLine* ep_line_2, const std::string& name, bool print, float size);
    void drawEpipolar(Image<colorRGB>* img, const colorRGB& color, int level=-1);

    bool searchMin(Candidate* candidate, Params* parameters);
    float getCost(const pixelIntensity magnitude3C_r, const pixelIntensity magnitude3C_m,
                  const pixelIntensity phase3C_r, const pixelIntensity phase3C_m,
                  const pixelIntensity color_r, const pixelIntensity color_m );
    float getCostNew(const pixelIntensity dh_r, const pixelIntensity dh_m,
                  const pixelIntensity dv_r, const pixelIntensity dv_m,
                  const pixelIntensity color_r, const pixelIntensity color_m );
    float getCostMagn(const pixelIntensity magnitude3C_r, const pixelIntensity magnitude3C_m,
                      const pixelIntensity color_r, const pixelIntensity color_m );
    float getCostPhase( const pixelIntensity phase3C_r, const pixelIntensity phase3C_m);

  private:

    friend class Mapper;

    void lineTraverse(int level);
    void coordToUV(float& coord, Eigen::Vector2f& uv);
    void UVToCoord(Eigen::Vector2f& uv, float& coord);

    // create imgs to show
    Image<colorRGB>* createEpipolarImg(int level=-1);
    Image<colorRGB>* createEpipolarImg(const std::string& name, int level=-1);

    Image<colorRGB>* createRangeStudyImg();


    // inline bool computeUOrV(Eigen::Vector2f& start_, Eigen::Vector2f& end_){
    //   float slope_ = (end_-start_).y()/(end_-start_).x();
    //   bool out = (slope<1 && slope>-1);
    //   return out;
    // }
    // inline float computeC0(Eigen::Vector2f& start_, Eigen::Vector2f& end_){
    //   float slope_ = (end_-start_).y()/(end_-start_).x();
    //   bool u_or_v_ = computeUOrV(start_, end_);
    //   float out = u_or_v_ ? start_.y()-slope*start_.x() : start_.x()-start_.y()/slope ;
    //   return out;
    // }
    inline float computeC0(Eigen::Vector2f& p, float slope_){
      bool u_or_v_ = (slope_<1 && slope_>-1);
      float out = u_or_v_ ? p.y()-slope_*p.x() : p.x()-p.y()/slope_ ;

      return out;
    }

};
