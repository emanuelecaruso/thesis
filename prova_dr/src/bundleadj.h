#pragma once
#include "camera.h"
#include "image.h"
#include "environment.h"
#include <stdexcept>

class Dtam; //forward declaration
class JacobiansAndError;


class deltaUpdateIncrements{
  public:
    Eigen::VectorXf* dx_poses;
    Eigen::VectorXf* dx_points;

    deltaUpdateIncrements(Eigen::VectorXf* dx_poses_, Eigen::VectorXf* dx_points_):
    dx_poses(dx_poses_),
    dx_points(dx_points_){}

    ~deltaUpdateIncrements(){
      delete dx_poses;
      delete dx_points;
    }
};

class JacobiansAndError{
  public:
    const Eigen::Matrix<float,1,6>* J_r;
    const Eigen::Matrix<float,1,6>* J_m;
    const float J_d;
    Eigen::Matrix<float,6,1>* J_r_transp;
    Eigen::Matrix<float,6,1>* J_m_transp;
    const float weight_total;

    const ActivePoint* active_pt;
    const CameraForMapping* cam_m;

    const float error;
    const float chi;
    const float omega;

    JacobiansAndError(Eigen::Matrix<float,1,6>* J_r_, Eigen::Matrix<float,1,6>* J_m_, float J_d_,
                      CameraForMapping* cam_m_, ActivePoint* active_pt_, float error_, float chi_,
                      float weight_total_, float omega_ ):
    J_r(J_r_),
    J_m(J_m_),
    J_d(J_d_),
    J_r_transp(new Eigen::Matrix<float,6,1>),
    J_m_transp(new Eigen::Matrix<float,6,1>),
    weight_total(weight_total_),

    active_pt(active_pt_),
    cam_m(cam_m_),

    error(error_),
    chi(chi_),
    omega(omega_)
    {
      if(J_r==nullptr){
        delete J_r_transp;
        J_r_transp= nullptr;
      }
      else
        *J_r_transp=J_r_->transpose();

      if(J_m==nullptr){
        delete J_m_transp;
        J_m_transp= nullptr;
      }
      else
        *J_m_transp=J_m_->transpose();

    }

    ~JacobiansAndError(){
      delete J_r;
      delete J_m;
      delete J_r_transp;
      delete J_m_transp;
    }

};

class HessianAndB_base{
public:

  // initialize just the class
  HessianAndB_base():
  pose_block_size(0),
  point_block_size(0),
  H_pose_pose(nullptr),
  H_pose_point(nullptr),
  H_point_pose(nullptr),
  H_point_point(nullptr),
  H_point_point_inv(nullptr),
  b_pose(nullptr),
  b_point(nullptr){}

  // clone class
  HessianAndB_base(HessianAndB_base* hessiand_and_b_ ):
    // block sizes
    pose_block_size(hessiand_and_b_->pose_block_size),
    point_block_size(hessiand_and_b_->point_block_size),

    // initialize H blocks
    H_pose_pose(hessiand_and_b_->H_pose_pose),
    H_pose_point(hessiand_and_b_->H_pose_point),
    H_point_pose(hessiand_and_b_->H_point_pose),
    H_point_point(hessiand_and_b_->H_point_point),
    H_point_point_inv(hessiand_and_b_->H_point_point_inv),

    // initialize b blocks
    b_pose(hessiand_and_b_->b_pose),
    b_point(hessiand_and_b_->b_point)
    { }

  // initialize H and b matrices knowing sizes
  HessianAndB_base(int pose_block_size_, int point_block_size_ ):
  // block sizes
  pose_block_size(pose_block_size_),
  point_block_size(point_block_size_),

  // initialize H blocks
  H_pose_pose(new Eigen::MatrixXf(pose_block_size_,pose_block_size_)),
  H_pose_point(new Eigen::MatrixXf(pose_block_size_,point_block_size_)),
  H_point_pose(new Eigen::MatrixXf(point_block_size_,pose_block_size_)),
  H_point_point(new Eigen::DiagonalMatrix<float,Eigen::Dynamic>(point_block_size_) ),
  H_point_point_inv(nullptr ),

  // initialize b blocks
  b_pose(new Eigen::VectorXf(pose_block_size_)),
  b_point(new Eigen::VectorXf(point_block_size_))
  {
    H_pose_pose->setZero();
    H_pose_point->setZero();
    H_point_pose->setZero();
    H_point_point->setZero();
    b_pose->setZero();
    b_point->setZero();
  }

  ~HessianAndB_base(){
    deleteAllPtrs();
  }

  inline bool isNull(){
    if (H_pose_pose==nullptr &&
        H_pose_point==nullptr &&
        H_point_pose==nullptr &&
        H_point_point==nullptr &&
        b_pose==nullptr &&
        b_point==nullptr ){
        return true;}
    else if ( H_pose_pose!=nullptr &&
              H_pose_point!=nullptr &&
              H_point_pose!=nullptr &&
              H_point_point!=nullptr &&
              b_pose!=nullptr &&
              b_point!=nullptr ){
        return false;}
    else{
      // return false;
      throw std::invalid_argument( "some pointers are null and some other not" );
    }

  }

  inline void initHessianAndB(int pose_block_size_, int point_block_size_){
    assert(pose_block_size_>=0);
    assert(point_block_size_>=0);

    // if(!this->isNull())
    //   this->deleteAllPtrs();

    // block sizes
    pose_block_size=pose_block_size_;
    point_block_size=point_block_size_;

    // initialize H blocks
    H_pose_pose= new Eigen::MatrixXf(pose_block_size_,pose_block_size_) ;
    H_pose_point= new Eigen::MatrixXf(pose_block_size_,point_block_size_);
    H_point_pose= new Eigen::MatrixXf(point_block_size_,pose_block_size_);
    H_point_point= new Eigen::DiagonalMatrix<float,Eigen::Dynamic>(point_block_size_) ;

    // initialize b blocks
    b_pose = new Eigen::VectorXf(pose_block_size_);
    b_point = new Eigen::VectorXf(point_block_size_);

    H_pose_pose->setZero();
    H_pose_point->setZero();
    H_point_pose->setZero();
    H_point_point->setZero();
    b_pose->setZero();
    b_point->setZero();
  }

  inline void deleteAllPtrs(){
    if (H_pose_pose!=nullptr)
      delete H_pose_pose;
    if (H_pose_point!=nullptr)
      delete H_pose_point;
    if (H_point_pose!=nullptr)
      delete H_point_pose;
    if (H_point_point!=nullptr)
      delete H_point_point;
    if (b_pose!=nullptr)
      delete b_pose;
    if (b_point!=nullptr)
      delete b_point;
  }

  Eigen::DiagonalMatrix<float,Eigen::Dynamic>* invertHPointPoint();
  Eigen::DiagonalMatrix<float,Eigen::Dynamic>* invertHPointPointDLS(float mu);

  void LMDampening(Params* parameters);

  void mirrorTriangH(bool pose_pose_not_diagonal=true);
  bool visualizeH(const std::string& name);

  int pose_block_size;
  int point_block_size;
  Eigen::MatrixXf* H_pose_pose;
  Eigen::MatrixXf* H_pose_point;
  Eigen::MatrixXf* H_point_pose;
  Eigen::DiagonalMatrix<float,Eigen::Dynamic>* H_point_point;
  Eigen::DiagonalMatrix<float,Eigen::Dynamic>* H_point_point_inv;
  Eigen::VectorXf* b_pose;
  Eigen::VectorXf* b_point;

  Eigen::MatrixXf* getSchurHposepose();
  void resetInv();

};

class HessianAndB_Marg;

class HessianAndB : public HessianAndB_base{
  public:

    HessianAndB(int pose_block_size_, int point_block_size_ ):
    HessianAndB_base(pose_block_size_, point_block_size_){}

    bool updateHessianAndB(JacobiansAndError* jacobians_and_error );

    deltaUpdateIncrements* getDeltaUpdateIncrements();
    deltaUpdateIncrements* getDeltaUpdateIncrementsProva(HessianAndB_Marg* hessian_b_marg);
    deltaUpdateIncrements* getDeltaUpdateIncrementsOnlyPoints();
    deltaUpdateIncrements* getDeltaUpdateIncrementsOnlyPoses();
    deltaUpdateIncrements* getDeltaUpdateIncrements_Slow();

};

class HessianAndB_Marg : public HessianAndB_base{
  public:

    // HessianAndB_Marg(int pose_block_size_, int point_block_size_ ):
    // HessianAndB_base(pose_block_size_, point_block_size_){}

    HessianAndB_Marg( ):
    HessianAndB_base(),
    hessian_b_marg_old(nullptr)
    {}

    void updateHessianAndB_marg(JacobiansAndError* jacobians_and_error );
    void updateBFromDelta(Eigen::VectorXf* dx_poses_marg);

    HessianAndB_base* hessian_b_marg_old;

    void curr2old();
    bool visualizeHMarg(const std::string& name);

    Eigen::MatrixXf* getSchurB();


};

class priorMarg{
  public:
    std::vector<CameraForMapping*>* activeKeyframes;
    std::vector<ActivePoint*>* marginalizedPoint;
    bool active;

    HessianAndB* hessian_b;

    // default constructor
    priorMarg(int n_cams, int n_points_marg ):
      activeKeyframes(new std::vector<CameraForMapping*>),
      marginalizedPoint(new std::vector<ActivePoint*>),
      active(false),
      hessian_b(new HessianAndB(n_cams,n_points_marg))
    {}

    void build(int n_cams, int n_points_marg, std::vector<JacobiansAndError*>* jacobians_and_error_vec);
};

class BundleAdj{

  public:
    BundleAdj(Dtam* dtam, Params* parameters):
    debug_optimization_(false),
    dtam_(dtam),
    parameters_(parameters),
    keyframe_vector_ba_(new std::vector<int>),
    hessian_b_marg(new HessianAndB_Marg),
    frame_current_ba(-1),
    num_active_points_(0),
    opt_norm_(HUBER),
    test_single_(TEST_ALL),
    image_id_(INTENSITY_ID),
    test_marginalization_(false),
    min_num_of_active_pts_per_region_(INT_MAX),
    chi_history(new std::vector<float>),
    pose_angle_error_history(new std::vector<float>),
    pose_position_error_history(new std::vector<float>),
    points_error_history(new std::vector<float>)
    {};


    bool projectActivePoints_prepMarg(bool take_fixed_point=false);
    void projectActivePoints(CameraForMapping* cam, bool take_fixed_point=false);

    void updateCurrentGuess();
    void updateActivePointsAfterNewPose();
    void updateActivePointsAfterNewPose(CameraForMapping* keyframe);


    ActivePointProjected* projectActivePoint(ActivePoint* active_pt, CamCouple* cam_couple);
    void activateNewPoints();
    void getCoarseActivePoints();
    void collectCoarseActivePoints();

    float getWeightTotal(float error);

    // JacobiansAndError* getJacobiansAndError(ActivePoint* active_pt, CameraForMapping* cam_m, bool no_r=false);
    JacobiansAndError* getJacobiansAndError(ActivePoint* active_pt, CameraForMapping* cam_m, bool no_r=false);
    std::vector<int> collectImageIds();
    void initializeStateStructure( int& n_cams, int& n_points, std::vector<JacobiansAndError*>* jacobians_and_error_vec );
    // void initializeStateStructure_onlyM( int& n_cams, int& n_points, std::vector<JacobiansAndError*>* jacobians_and_error_vec );
    // void initializeStateStructure_onlyR( int& n_cams, int& n_points, std::vector<JacobiansAndError*>* jacobians_and_error_vec );
    // void initializeStateStructure_onlyD( int& n_cams, int& n_points, std::vector<JacobiansAndError*>* jacobians_and_error_vec );
    // void initializeStateStructure_onlyMandPoints( int& n_cams, int& n_points, std::vector<JacobiansAndError*>* jacobians_and_error_vec );

    void initializeStateStructureMarg( int& n_cams, int& n_points, std::vector<JacobiansAndError*>* jacobians_and_error_vec, CameraForMapping* keyframe_to_marginalize );

    bool updateOldMargHessianAndB();
    void getJacobiansForNewUpdate(int& new_points, int& poses_new_size, std::vector<JacobiansAndError*>* jacobians_and_error_vec );
    void updateMargHessianAndB(int new_points, int poses_new_size, std::vector<JacobiansAndError*>* jacobians_and_error_vec);

    void deleteMarginalizedPoints();
    void removeMarginalizedKeyframe();
    bool marginalization();
    bool detectOutliers();

    priorMarg* updateMarginalizationPrior( int n_cams, int n_points_marg, std::vector<JacobiansAndError*>* jacobians_and_error_vec);
    bool updateBForMarg(deltaUpdateIncrements* delta);
    Eigen::VectorXf* getDeltaForMargFromOpt(deltaUpdateIncrements* delta);

    float optimizationStep(bool with_marg=true);
    float getChi(float error, float omega=1);
    void optimize();

    inline void addKeyframe(int idx){
      keyframe_vector_ba_->push_back(idx);
    }

    inline int getFrameCurrentIdxBA(){
      return frame_current_ba;
    }

    CameraForMapping* getFrameCurrentBA();
    bool debug_optimization_;
    std::vector<int>* keyframe_vector_ba_;
    HessianAndB_Marg* hessian_b_marg;
    int frame_current_ba;
    int opt_norm_;
    int test_single_;
    int image_id_;
    bool test_marginalization_;

  private:

    friend class Tracker;

    Dtam* const dtam_;
    Params* const parameters_;
    int num_active_points_;
    int min_num_of_active_pts_per_region_;
    std::vector<float>* chi_history;
    std::vector<float>* pose_angle_error_history;
    std::vector<float>* pose_position_error_history;
    std::vector<float>* points_error_history;

    ActivePoint* activateCandidate(CandidateProjected* cand_proj, RegionWithProjCandidates* reg, RegionsWithProjActivePoints* regs);
    void addCoarseActivePointInRegion(ActivePoint* active_pt);
    void removeCoarseActivePointsInRegion(ActivePoint* active_pt);

    // void projectCandidates(CameraForMapping* keyframe, CameraForMapping* new_keyframe);

    void updateDeltaUpdates(deltaUpdateIncrements* delta);
    void updateDeltaUpdatesOnlyD(deltaUpdateIncrements* delta);
    void updateInvdepthVars(HessianAndB* hessian_and_b);
    void updateTangentSpace(bool with_marg);
    void fixNewTangentSpaceOnlyD();

    Eigen::Matrix<float,1,2> getImageJacobian(ActivePoint* active_pt, CameraForMapping* cam_m, pxl& pixel_m, int image_);
    Eigen::Matrix<float, 2,3>* getJfirst_(ActivePoint* active_pt, CameraForMapping* cam_m, Eigen::Vector3f& point_m_0, pxl& pixel_m);
    Eigen::Matrix<float,1,3> getJfirst(Eigen::Matrix<float, 2,3>* Jfirst_, Eigen::Matrix<float,1,2> img_jacobian );
    Eigen::Matrix3f getRelativeRotationMatrix(ActivePoint* active_pt, CameraForMapping* cam_m);
    Eigen::Matrix<float,3,6> getJSecondJr(ActivePoint* active_pt, Eigen::Matrix3f relative_rot_mat );
    Eigen::Matrix<float,3,6> getJSecondJm( Eigen::Vector3f& point_m_0 );
    Eigen::Matrix<float,3,1> getJSecondJd( ActivePoint* active_pt, Eigen::Matrix3f relative_rot_mat );
    Eigen::Matrix<float,1,6> getJr( Eigen::Matrix<float,1,3> J_first, Eigen::Matrix<float,3,6> JSecond_jr );
    Eigen::Matrix<float,1,6> getJm( Eigen::Matrix<float,1,3> J_first, Eigen::Matrix<float,3,6> JSecond_jm);
    float getJd( Eigen::Matrix<float,1,3> J_first, Eigen::Matrix<float,3,1> JSecond_jd );

    float getError(ActivePoint* active_pt, CameraForMapping* cam_m, pxl& pixel_m, int image_id);

};
