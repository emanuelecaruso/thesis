#pragma once
#include "camera.h"
#include "epline.h"

class Dtam; //forward declaration

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

    float coord2FromCoord1(float coord1, Eigen::Vector4f& abcd);

    void getParametersABCD( EpipolarLine* ep_line_source, EpipolarLine* ep_line_range,
                                float depth, Eigen::Vector4f& abcd);

    void showRangeStudy(EpipolarLine* ep_line_source, EpipolarLine* ep_line_range,
                            int uvs_idx, float size=1);

};
