#pragma once
#include "camera.h"
#include "image.h"

class Dtam; //forward declaration

class EpipolarLine{
  public:
    // line parameters
    const float slope;  // slope of the line
    const float vh;     // y value at x=0

    // defines wether start and end are expressed as  u or v coordinate, true->u, false->v
    const bool u_or_v;
    // slope is kept const, start and end can be moved
    float start;  // start of segment
    float end;    // end of segment

    const Camera* cam; // camera associated to epipolar line
    std::vector<Eigen::Vector2f>* uvs;  // vector of interpoled uvs along epipolar line

    EpipolarLine( const Camera* cam_, Eigen::Vector2f& start_, Eigen::Vector2f& end_):
    slope((end_-start_).y()/(end_-start_).x()),
    vh(start_.y()-slope*start_.x()),
    u_or_v( (slope<1 && slope>-1) ),
    cam(cam_)
    {
      UVToCoord(start_,start);
      UVToCoord(end_,end);
    }

    void printMembers() const;
    bool resizeLine();
    void showEpipolar(int size);
  private:
    void coordToUV(float& coord, Eigen::Vector2f& uv);
    void UVToCoord(Eigen::Vector2f& uv, float& coord);
    void lineTraverse();

};

struct featureData{
  int idx;
  cv::Vec3b color;
  cv::Vec3b gradient;
  int parent = -1;
  float upperbound; //  in u coord
  float lowerbound; //  in u coord

  inline void printMembers(){
    sharedCout("idx: "+std::to_string(idx));
    sharedCout("parent: "+std::to_string(parent));
    sharedCout("upperbound: "+std::to_string(upperbound));
    sharedCout("lowerbound: "+std::to_string(lowerbound));
    sharedCout("color: "+std::to_string(color[0])+
                    ", "+std::to_string(color[1])+
                    ", "+std::to_string(color[2]));
    sharedCout("gradient: "+std::to_string(gradient[0])+
                       ", "+std::to_string(gradient[1])+
                       ", "+std::to_string(gradient[2]));
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

    bool computeEpipolarLineCouple(const Camera* cam_1, const Camera* cam_2,
                                Eigen::Vector2f& uv_1, EpipolarLine*& ep_line_1,
                                EpipolarLine*& ep_line_2);

};
