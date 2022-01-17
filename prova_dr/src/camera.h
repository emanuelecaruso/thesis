#pragma once
#include "defs.h"
#include "parameters.h"
#include "json.hpp"
// #include "image.h"
#include "wavelet.h"
#include <sys/stat.h>

using namespace pr;

typedef std::pair<float,float> bound;

class Camera{
  public:

    const std::string name_;
    const CamParameters* cam_parameters_;
    const Eigen::Matrix3f* K_;
    const Eigen::Matrix3f* Kinv_;
    const Image<pixelIntensity>* image_intensity_;
    Image<float>* invdepth_map_;
    Eigen::Isometry3f* frame_camera_wrt_world_;
    Eigen::Isometry3f* frame_world_wrt_camera_;


    Camera(const std::string& name, const CamParameters* cam_parameters,
           const Image<pixelIntensity>* image_intensity):
           name_(name),
           cam_parameters_(cam_parameters),
           K_(compute_K()),
           Kinv_( new Eigen::Matrix3f(K_->inverse()) ),
           image_intensity_( image_intensity ),
           frame_camera_wrt_world_(new Eigen::Isometry3f),
           frame_world_wrt_camera_(new Eigen::Isometry3f)
           { };

     Camera(const std::string& name, const CamParameters* cam_parameters,
            const std::string& path_rgb):
            name_(name),
            cam_parameters_(cam_parameters),
            K_(compute_K()),
            Kinv_( new Eigen::Matrix3f(K_->inverse()) ),
            image_intensity_( returnIntensityImgFromPath( path_rgb ) ),
            frame_camera_wrt_world_(new Eigen::Isometry3f),
            frame_world_wrt_camera_(new Eigen::Isometry3f)
            { };

    Camera(const std::string& name, const CamParameters* cam_parameters,
           const Image<pixelIntensity>* image_intensity, Eigen::Isometry3f* frame_world_wrt_camera,
               Eigen::Isometry3f* frame_camera_wrt_world ):
           Camera(name, cam_parameters, image_intensity )
           {
             frame_world_wrt_camera_=frame_world_wrt_camera;
             frame_camera_wrt_world_=frame_camera_wrt_world;
           };

    Camera(const std::string& name, const CamParameters* cam_parameters,
           nlohmann::basic_json<>::value_type f,
           const std::string& path_rgb):
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
    void getCenterAsUV(Eigen::Vector2f& uv) const;
    void getCentreAsPixel(Eigen::Vector2i& pixel_coords) const;
    float getPixelWidth(int level=-1) const;

    // assign
    void assignPose(Eigen::Isometry3f& frame_camera_wrt_world);

    // functions for projections/transformations
    void pixelCoords2uv(const Eigen::Vector2i& pixel_coords, Eigen::Vector2f& uv, int level) const;
    void pixelCoords2uv(const Eigen::Vector2i& pixel_coords, Eigen::Vector2f& uv) const;
    void uv2pixelCoords(const Eigen::Vector2f& uv, Eigen::Vector2i& pixel_coords, int level) const;
    void uv2pixelCoords(const Eigen::Vector2f& uv, Eigen::Vector2i& pixel_coords) const;
    void pointAtDepth(const Eigen::Vector2f& uv, float depth, Eigen::Vector3f& p) const;
    bool projectPoint(const Eigen::Vector3f& p, Eigen::Vector2f& uv, float& p_cam_z ) const;
    bool projectPoint(const Eigen::Vector3f& p, Eigen::Vector2f& uv) const;
    bool projectCam(const Camera* cam_to_be_projected, Eigen::Vector2f& uv) const;
    bool projectCam(const Camera* cam_to_be_projected, Eigen::Vector2f& uv, float& p_cam_z) const;

    // functions for images
    void clearImgs();
    void saveRGB(const std::string& path) const;
    void saveDepthMap(const std::string& path) const;
    void showRGB(int image_scale=1) const;
    void showDepthMap(int image_scale=1) const;


    inline Camera* clone(){
      return new Camera(*this);
    }
  protected:
    Eigen::Matrix3f* compute_K();
    Image<pixelIntensity>* returnIntensityImgFromPath(const std::string& path_rgb);
    void loadWhiteDepth();
    void loadDepthMap(const std::string& path);
    void loadPoseFromJsonVal(nlohmann::basic_json<>::value_type f);



};


class Mapper; // forward declaration

class CameraForMapping;


class CandidateBase{
  public:
    const int level_;
    const Eigen::Vector2i pixel_;
    const Eigen::Vector2f uv_;

    CandidateBase(const int level, const Eigen::Vector2i& pixel, const Eigen::Vector2f& uv):
    level_(level), pixel_(pixel), uv_(uv){};
};

class Candidate;

class RegionWithCandidatesBase{
  public:

    int x_, y_, reg_level_;
    float grad_threshold_;
    CameraForMapping* cam_;

    RegionWithCandidatesBase(int x, int y, Params* parameters, CameraForMapping* cam):
    x_(x), y_(y), reg_level_(parameters->reg_level),
    grad_threshold_(parameters->grad_threshold), cam_(cam)
    {    };
    void showRegion(int size);

};

class RegionWithCandidates : public RegionWithCandidatesBase{
  public:

    std::vector<Candidate*>* cands_vec_;

    RegionWithCandidates(int x, int y, Params* parameters, CameraForMapping* cam):
    RegionWithCandidatesBase( x, y, parameters, cam),
    cands_vec_(new std::vector<Candidate*>){
      // collectCandidates();
    };
    bool collectCandidates(int wavelet_levels);

};

class Candidate : public CandidateBase{
  public:


    Candidate(int level, Eigen::Vector2i& pixel, Eigen::Vector2f& uv,
              float grad_magnitude,
              pixelIntensity dh, pixelIntensity dv, pixelIntensity intensity,
              std::vector<bound>* bounds, RegionWithCandidates* region ):
    CandidateBase( level, pixel, uv),
    bounds_(bounds),
    depth_(-1),
    region_(region),
    grad_magnitude_(grad_magnitude),
    dh_(dh),
    dv_(dv),
    intensity_(intensity)
    {};

    ~Candidate(){
      delete bounds_;
    };

    std::vector<bound>* bounds_;
    float depth_;
    RegionWithCandidates* region_;
    const float grad_magnitude_;
    const pixelIntensity dh_;
    const pixelIntensity dv_;
    pixelIntensity dh_robust_;
    pixelIntensity dv_robust_;
    const pixelIntensity intensity_;

    inline float getInvdepthVar() const{
      float d2 = bounds_->at(0).second;
      float d1 = bounds_->at(0).first;
      return (1.0/d1)-(1.0/d2);
    }

    inline void marginalize() const {
      std::vector<Candidate*>* v = region_->cands_vec_;
      v->erase(std::remove(v->begin(), v->end(), this), v->end());
      delete this;
    }
};



class CandidateProjected : public CandidateBase{
  public:
    const float invdepth_;
    const Candidate* cand_;

    CandidateProjected(Candidate* cand, Eigen::Vector2i& pixel, Eigen::Vector2f& uv, float invdepth):
    CandidateBase(cand->level_, pixel, uv),
    invdepth_(invdepth),
    cand_(cand)
    {};


};

class ActivePoint : public CandidateBase{
  public:
    const float invdepth_;
    const float invdepth_var_;

    ActivePoint(CandidateProjected* cand_proj):
    CandidateBase( cand_proj->cand_->level_, cand_proj->cand_->pixel_, cand_proj->cand_->uv_),
    invdepth_(cand_proj->invdepth_),
    invdepth_var_( cand_proj->cand_->getInvdepthVar() )
    {}



};

class RegionsWithCandidatesBase{
  public:

    RegionsWithCandidatesBase(Params* parameters, CameraForMapping* cam, int res_x, int res_y):
    cam_(cam),
    parameters_(parameters),
    num_of_regions_x(res_x/pow(2,parameters->reg_level+1)),
    num_of_regions_y(res_y/pow(2,parameters->reg_level+1))
    {  };
    const CameraForMapping* cam_;
    const Params* parameters_;
    const int num_of_regions_x;
    const int num_of_regions_y;

    inline int xyToIdx(int x, int y){
      return (num_of_regions_x*y+x);
    }
};




class RegionsWithCandidates : public RegionsWithCandidatesBase{
  public:

    RegionsWithCandidates(Params* parameters, CameraForMapping* cam, int res_x, int res_y):
    RegionsWithCandidatesBase( parameters, cam, res_x, res_y),
    region_vec_(new std::vector<RegionWithCandidates*>(num_of_regions_x*num_of_regions_y) )
    {
      for(int x=0; x<num_of_regions_x; x++){
        for(int y=0; y<num_of_regions_y; y++){
          int idx=xyToIdx(x,y);
          region_vec_->at(idx)=new RegionWithCandidates(x, y, parameters, cam);
        }
      }
    };
    std::vector<RegionWithCandidates*>* region_vec_;

    inline void collectCandidates(int wavelet_levels){
      for (RegionWithCandidates* reg : *region_vec_ ){
        reg->collectCandidates(wavelet_levels);
        // break;
      }
    }

};


class RegionWithProjCandidates : public RegionWithCandidatesBase{
  public:

    std::vector<CandidateProjected*>* cands_vec_;

    RegionWithProjCandidates(int x, int y, Params* parameters, CameraForMapping* cam):
    RegionWithCandidatesBase( x, y, parameters, cam),
    cands_vec_(new std::vector<CandidateProjected*>){
    };

};

class RegionsWithProjCandidates : public RegionsWithCandidatesBase{
  public:

    RegionsWithProjCandidates(Params* parameters, CameraForMapping* cam, int res_x, int res_y):
    RegionsWithCandidatesBase( parameters, cam, res_x, res_y),
    region_vec_(new std::vector<RegionWithProjCandidates*>(num_of_regions_x*num_of_regions_y) )
    {
      for(int x=0; x<num_of_regions_x; x++){
        for(int y=0; y<num_of_regions_y; y++){
          int idx=xyToIdx(x,y);
          region_vec_->at(idx)=new RegionWithProjCandidates(x, y, parameters, cam);
        }
      }
    };
    std::vector<RegionWithProjCandidates*>* region_vec_;

    inline void pushCandidate(CandidateProjected* projected_cand){
      // get associated region
      int scale_offs = pow(2,parameters_->reg_level-projected_cand->level_);
      // int scale_offs = 1;
      int reg_x = projected_cand->pixel_.x()/scale_offs;
      int reg_y = projected_cand->pixel_.y()/scale_offs;
      int idx = xyToIdx( reg_x, reg_y);

      // push the projected candidate inside the region (sorted by invdepth var)

      std::vector<CandidateProjected*>* cands_vec_ = region_vec_->at(idx)->cands_vec_;

      auto lb_cmp = [](CandidateProjected* const & x, float d) -> bool
        { return x->cand_->getInvdepthVar() < d; };

      auto it = std::lower_bound(cands_vec_->begin(), cands_vec_->end(), projected_cand->cand_->getInvdepthVar(), lb_cmp);
      cands_vec_->insert ( it , projected_cand );
    }
};

class CameraForMapping: public Camera{

  public:


    Wvlt_dec* wavelet_dec_;
    std::vector<Candidate*>* candidates_;
    std::vector<ActivePoint*>* active_points_;
    std::vector<ActivePoint*>* marginalized_points_;
    RegionsWithCandidates* regions_;
    RegionsWithProjCandidates* regions_projected_cands_;
    int n_candidates_;
    friend class Mapper;
    friend class Dtam;

    CameraForMapping(const std::string& name, const CamParameters* cam_parameters,
           const Image<pixelIntensity>* image_intensity, Params* parameters):
           Camera( name, cam_parameters, image_intensity),
           wavelet_dec_(new Wvlt_dec(parameters->coarsest_level,new Image<pixelIntensity>(image_intensity_), this)),
           candidates_(new std::vector<Candidate*>),
           active_points_(new std::vector<ActivePoint*>),
           marginalized_points_(new std::vector<ActivePoint*>),
           regions_(new RegionsWithCandidates(parameters, this,
                      cam_parameters->resolution_x, cam_parameters->resolution_y)),
           regions_projected_cands_(new RegionsWithProjCandidates(parameters, this,
                      cam_parameters->resolution_x, cam_parameters->resolution_y)),
           n_candidates_(0)
           {};

    CameraForMapping(const std::string& name, const CamParameters* cam_parameters,
            const Image<pixelIntensity>* image_intensity, Eigen::Isometry3f* frame_world_wrt_camera,
                Eigen::Isometry3f* frame_camera_wrt_world, Params* parameters):
            CameraForMapping( name, cam_parameters, image_intensity, parameters)
            {
              frame_world_wrt_camera_=frame_world_wrt_camera;
              frame_camera_wrt_world_=frame_camera_wrt_world;
            };

    CameraForMapping(Camera* cam, Params* parameters):
           CameraForMapping( cam->name_, cam->cam_parameters_, cam->image_intensity_, parameters)
           {  };

  private:
    // void collectRegions(float grad_threshold);
    void selectNewCandidates(int max_num_candidates);
    void marginalizeActivePoint();
    void selectActivePoints(int max_num_active_points);
    void showCandidates_1(float size);
    void showCandidates_2(float size);
    // void showProjCandidates_1(float size);
    void showProjCandidates_2(float size);
    // void showActivePoints_1(float size);
    void showActivePoints_2(float size);
};
