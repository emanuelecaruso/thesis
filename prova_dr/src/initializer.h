#pragma once
#include "camera.h"
#include "image.h"
#include "environment.h"

class Dtam; //forward declaration


class Initializer{

  public:
    Initializer(Dtam* dtam, Params* parameters):
    dtam_(dtam),
    parameters_(parameters),
    corners_vec_(new std::vector<std::vector<cv::Point2f>*>),
    errors_vec_(new std::vector<std::vector<float>*>),
    status_vec_(new std::vector<std::vector<uchar>*>)
    {};

    void extractCorners();
    void showCornersRef();
    void trackCornersLK();
    void showCornersTrack();

  private:
    Dtam* const dtam_;
    Params* const parameters_;

    int ref_frame_idx_;
    std::vector<std::vector<cv::Point2f>*>* corners_vec_;
    std::vector<std::vector<uchar>*>* status_vec_;
    std::vector<std::vector<float>*>* errors_vec_;

    const Image<float>* getReferenceImage();
    const Image<float>* getCurrentImage();
    const Image<float>* getPrevImage();
};
