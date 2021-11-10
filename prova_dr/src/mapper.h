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
    bool resizeLine();
    void showEpipolar(float size);
    void showEpipolarComparison(EpipolarLine* ep_line_2, float size);
  private:
    Image<cv::Vec3b>* createEpipolarImg();
    void coordToUV(float& coord, Eigen::Vector2f& uv);
    void UVToCoord(Eigen::Vector2f& uv, float& coord);
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

};

class Feature{
  public:
    const int idx_;
    const cv::Vec3b color_;
    const cv::Vec3b gradient_;
    const float upperbound_; //  in u coord
    const float lowerbound_; //  in u coord
    int parent_ = -1;

    Feature(int idx, cv::Vec3b& color, cv::Vec3b& gradient,
            float upperbound, float lowerbound):
            idx_(idx), color_(color), gradient_(gradient),
            upperbound_(upperbound),
            lowerbound_(lowerbound)
            {}


    inline void printMembers(){
      sharedCout("idx: "+std::to_string(idx_));
      sharedCout("parent: "+std::to_string(parent_));
      sharedCout("upperbound: "+std::to_string(upperbound_));
      sharedCout("lowerbound: "+std::to_string(lowerbound_));
      sharedCout("color: "+std::to_string(color_[0])+
                      ", "+std::to_string(color_[1])+
                      ", "+std::to_string(color_[2]));
      sharedCout("gradient: "+std::to_string(gradient_[0])+
                         ", "+std::to_string(gradient_[1])+
                         ", "+std::to_string(gradient_[2]));
    }

};

class Mapper{

  public:
    Mapper(Dtam* dtam): dtam_(dtam){};

    void doMapping();

  private:
    Dtam* const dtam_;

    // frame coupling
    void frameCouplingRandom(int& frame_1, int& frame_2);
    void frameCouplingLast(int& frame_1, int& frame_2);
    void frameCouplingOpposite(int& frame_1, int& frame_2);

    bool computeEpipolarLineCouple(const Camera* cam_1, const Camera* cam_2,
                                Eigen::Vector2f& uv_1, EpipolarLine*& ep_line_1,
                                EpipolarLine*& ep_line_2);

    bool buildFeatureVec(EpipolarLine*& ep_line_1, EpipolarLine*& ep_line_2,
                std::vector<Feature*>*& feats_1, std::vector<Feature*>*& feats_2);

    void getParametersABCD(EpipolarLine* ep_line_1, EpipolarLine* ep_line_2,
                          float& A,float& B,float& C,float& D);

};
