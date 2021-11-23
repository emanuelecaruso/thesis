#pragma once
#include "camera.h"
#include "image.h"

class Dtam; //forward declaration




class EpipolarLine{
  public:

    // line parameters

    // defines wether start, end and x0 are expressed as  u or v coordinate, true->u, false->v
    const bool u_or_v;
    const float slope;  // slope of the line
    const float c0;     // v at u=0, or u at v=0

    // slope is kept const, start and end can be moved
    float start;  // start of segment (u or v coord)
    float end;    // end of segment (u or v coord)

    const Camera* cam; // camera associated to epipolar line
    std::vector<Eigen::Vector2f>* uvs;  // vector of interpoled uvs along epipolar line

    EpipolarLine( const Camera* cam_, float slope_, float c_start_, float c_end_, float c0_ ):
    slope( slope_ ),
    u_or_v( (slope_<1 && slope_>-1) ),
    c0( c0_ ),
    cam(cam_)
    {
      start=c_start_;
      end=c_end_;
    }

    EpipolarLine( const Camera* cam_, float slope_, float c_start_,
                  float c_end_, Eigen::Vector2f& cam_proj_ ):
    EpipolarLine( cam_, slope_, c_start_, c_end_, computeC0( cam_proj_, slope_)  ){}


    EpipolarLine( const Camera* cam_, Eigen::Vector2f& start_, Eigen::Vector2f& end_):
    slope( (end_-start_).y()/(end_-start_).x() ),
    u_or_v( computeUOrV(start_, end_) ),
    c0( computeC0(start_, end_) ),
    cam(cam_)
    {
      UVToCoord(start_,start);
      UVToCoord(end_,end);
    }

    void printMembers() const;
    bool stretchToBorders();
    bool resizeWithinImage();

    // show
    void showEpipolar(float size);
    void showEpipolarComparison(EpipolarLine* ep_line_2, float size);
    void showEpipolarComparison(EpipolarLine* ep_line_2, const std::string& name, float size);
    void showRangeStudy(EpipolarLine* ep_line_2, int uvs_idx, float size=1);


  private:

    friend class Mapper;

    void coordToUV(float& coord, Eigen::Vector2f& uv);
    void UVToCoord(Eigen::Vector2f& uv, float& coord);

    // create imgs to show
    Image<cv::Vec3b>* createEpipolarImg();
    Image<cv::Vec3b>* createEpipolarImg(const std::string& name);

    Image<cv::Vec3b>* createRangeStudyImg();


    void lineTraverse();
    inline bool computeUOrV(Eigen::Vector2f& start_, Eigen::Vector2f& end_){
      float slope_ = (end_-start_).y()/(end_-start_).x();
      bool out = (slope_<1 && slope_>-1);
      return out;
    }
    inline float computeC0(Eigen::Vector2f& start_, Eigen::Vector2f& end_){
      float slope_ = (end_-start_).y()/(end_-start_).x();
      bool u_or_v_ = computeUOrV(start_, end_);
      float out = u_or_v ? start_.y()-slope*start_.x() : start_.x()-start_.y()/slope ;
      return out;
    }
    inline float computeC0(Eigen::Vector2f& p, float slope_){
      bool u_or_v_ = (slope_<1 && slope_>-1);
      float out = u_or_v ? p.y()-slope_*p.x() : p.x()-p.y()/slope_ ;
      return out;
    }

};
