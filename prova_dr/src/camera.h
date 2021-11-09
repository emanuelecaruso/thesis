#pragma once
#include "defs.h"
#include "json.hpp"
#include "image.h"
#include <sys/stat.h>

using namespace pr;

struct Pyramid{
  public:
    Image<cv::Vec3b>* pyr_2;
    Image<cv::Vec3b>* pyr_4;
    Image<cv::Vec3b>* pyr_8;

    Pyramid(Image<cv::Vec3b>* image_rgb){
      // https://docs.opencv.org/3.4.15/d4/d1f/tutorial_pyramids.html TODO
    };
  // private:
  //   void makePyramid();

};

class Camera{
  public:

    const std::string name_;
    const CamParameters* cam_parameters_;
    const Eigen::Matrix3f* K_;
    const Eigen::Matrix3f* Kinv_;
    const Image<cv::Vec3b>* image_rgb_;
    const Image<cv::Vec3f>* features_;
    Image<float>* invdepth_map_;
    Eigen::Isometry3f* frame_world_wrt_camera_;
    Eigen::Isometry3f* frame_camera_wrt_world_;



    Camera(const std::string& name, const CamParameters* cam_parameters,
           const std::string& path_rgb ):
           name_(name),
           cam_parameters_(cam_parameters),
           K_(compute_K()),
           Kinv_( new Eigen::Matrix3f(K_->inverse()) ),
           image_rgb_( returnRGBFromPath( path_rgb ) ),
           features_( computeCurvature() ){};

    Camera(const std::string& name, const CamParameters* cam_parameters,
           nlohmann::basic_json<>::value_type f,
           const std::string& path_rgb ):
    Camera( name,cam_parameters, path_rgb )
    {
      loadPoseFromJsonVal(f);
      invdepth_map_ = new Image< float >("invdepth_"+name_);
      loadWhiteDepth();
    };

    Camera(const std::string& name, const CamParameters* cam_parameters,
           nlohmann::basic_json<>::value_type f,
           const std::string& path_rgb,  const std::string& path_depth ):
    Camera( name,cam_parameters, f, path_rgb )
    {
      loadDepthMap(path_depth);
    };



    void printMembers() const;

    // sampling
    void sampleRandomUv(Eigen::Vector2f& uv);
    void sampleRandomPixel(Eigen::Vector2i& pixel_coords);

    // access
    void getCentreAsUV(Eigen::Vector2f& uv);
    void getCentreAsPixel(Eigen::Vector2i& pixel_coords);

    // functions for projections/transformations
    void pixelCoords2uv(const Eigen::Vector2i& pixel_coords, Eigen::Vector2f& uv) const;
    void uv2pixelCoords(const Eigen::Vector2f& uv, Eigen::Vector2i& pixel_coords) const;
    void pointAtDepth(const Eigen::Vector2f& uv, float depth, Eigen::Vector3f& p) const;
    bool projectPoint(const Eigen::Vector3f& p, Eigen::Vector2f& uv, float& p_cam_z ) const;
    bool projectPoint(const Eigen::Vector3f& p, Eigen::Vector2f& uv) const;
    bool projectCam(const Camera* cam_to_be_projected, Eigen::Vector2f& uv) const;

    // functions for images
    void clearImgs();
    void saveRGB(const std::string& path) const;
    void saveDepthMap(const std::string& path) const;
    void showRGB(int image_scale=1) const;
    void showDepthMap(int image_scale=1) const;

    void showWorldFrame(Eigen::Vector3f& origin, float delta,int length);

    inline Camera* clone(){
      return new Camera(*this);
    }
  private:
    Eigen::Matrix3f* compute_K();
    Image<cv::Vec3b>* returnRGBFromPath(const std::string& path_rgb);
    void loadWhiteDepth();
    void loadDepthMap(const std::string& path);
    void loadPoseFromJsonVal(nlohmann::basic_json<>::value_type f);

    //feature types
    Image<cv::Vec3f>* computeCurvature();
    Image<cv::Vec6f>* derivativeXY();

};
