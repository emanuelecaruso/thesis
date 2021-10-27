#pragma once
#include "defs.h"
#include "image.h"

using namespace pr;


class Camera{
  public:

    const std::string name_;
    const CamParameters* cam_parameters_;
    const Eigen::Matrix3f* K_;
    const Eigen::Matrix3f* Kinv_;
    Image<float>* invdepth_map_;
    Image<cv::Vec3b>* image_rgb_;
    Eigen::Isometry3f* frame_world_wrt_camera_;
    Eigen::Isometry3f* frame_camera_wrt_world_;

    Camera(const std::string& name, const CamParameters* cam_parameters, Eigen::Isometry3f* frame_camera_wrt_world):
    name_(name),
    cam_parameters_(cam_parameters),
    K_(compute_K()),
    Kinv_( new Eigen::Matrix3f(K_->inverse()) )

    {

       float resolution_x = cam_parameters->resolution_x;
       float resolution_y = cam_parameters->resolution_y;
       float max_depth = cam_parameters->max_depth;
       float min_depth = cam_parameters->min_depth;

       Eigen::Isometry3f* frame_world_wrt_camera = new Eigen::Isometry3f;
       *frame_world_wrt_camera=frame_camera_wrt_world->inverse();

       frame_camera_wrt_world_ = frame_camera_wrt_world;
       frame_world_wrt_camera_ = frame_world_wrt_camera;

       invdepth_map_ = new Image< float >("invdepth_"+name_);
       image_rgb_ = new Image< cv::Vec3b >("rgb_"+name_);

       // initialize images with white color
       invdepth_map_->initImage(resolution_y,resolution_x);
       invdepth_map_->setAllPixels(1.0);
       image_rgb_->initImage(resolution_y,resolution_x);
       image_rgb_->setAllPixels(cv::Vec3b(255,255,255));

    };

    void printMembers() const;

    // functions for projections/transformations
    void pixelCoords2uv(Eigen::Vector2i& pixel_coords, Eigen::Vector2f& uv) const;
    void uv2pixelCoords( Eigen::Vector2f& uv, Eigen::Vector2i& pixel_coords) const;
    void pointAtDepth(Eigen::Vector2f& uv, float depth, Eigen::Vector3f& p) const;
    bool projectPoint(Eigen::Vector3f& p, Eigen::Vector2f& uv, float& p_cam_z ) const;

    // functions for images
    void clearImgs();
    void saveRGB(std::string path) const;
    void saveDepthMap(std::string path) const;
    void loadRGB(std::string path);
    void loadDepthMap(std::string path);
    void showRGB(int image_scale=1) const;
    void showDepthMap(int image_scale=1) const;

    void showWorldFrame(Eigen::Vector3f origin, float delta,int length);

    inline Camera* clone(){
      return new Camera(*this);
    }
  private:
    Eigen::Matrix3f* compute_K();

};

typedef std::vector<Camera*> CameraVector;
