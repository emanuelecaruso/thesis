#include "mapper.cuh"
#include <math.h>
#include "utils.h"
#include <stdlib.h>
#include "defs.h"
#include "cuda_utils.cuh"




__global__ void prepareCameraForDtam_kernel(Camera_gpu* camera_r, Camera_gpu* camera_m, cv::cuda::PtrStepSz<float3> query_proj_matrix){
  int row = blockIdx.x * blockDim.x + threadIdx.x;
  int col = blockIdx.y * blockDim.y + threadIdx.y;

  Eigen::Vector2i pixel_coords_r(col,row);

  // query point
  Eigen::Vector3f query_p;
  Eigen::Vector2f uv_r;
  camera_r->pixelCoords2uv(pixel_coords_r, uv_r);
  camera_r->pointAtDepth(uv_r, camera_r->max_depth_, query_p);

  // project query point
  Eigen::Vector2f query_p_projected_on_cam_m;
  float query_depth_on_camera_m;
  bool query_in_front = camera_m->projectPoint(query_p, query_p_projected_on_cam_m, query_depth_on_camera_m);

  float3 val;
  if (!query_in_front)
    val = make_float3( -1,-1,-1 );
  else
    val = make_float3( query_p_projected_on_cam_m.x(), query_p_projected_on_cam_m.y(), query_depth_on_camera_m );

  query_proj_matrix(row,col)=val;

}


void Mapper::prepareCameraForDtam(int index_m){
  Camera_cpu* camera_r = camera_vector_cpu_[index_r_];
  Eigen::Vector3f camera_r_p = camera_r->frame_camera_wrt_world_->translation();
  Camera_cpu* camera_m = camera_vector_cpu_[index_m];

  int cols = camera_r->invdepth_map_->image_.cols;
  int rows = camera_r->invdepth_map_->image_.rows;

  // project camera_r on camera_m
  Eigen::Vector2f cam_r_projected_on_cam_m;
  float cam_r_depth_on_camera_m;
  bool cam_r_in_front = camera_m->projectPoint(camera_r_p, cam_r_projected_on_cam_m, cam_r_depth_on_camera_m);

  Eigen::Isometry3f T = (*(camera_m->frame_world_wrt_camera_))*(*(camera_r->frame_camera_wrt_world_));
  Eigen::Matrix3f r=T.linear();
  Eigen::Vector3f t=T.translation();

  cameraDataForDtam* camera_data_for_dtam_h = new cameraDataForDtam;
  camera_data_for_dtam_h->T_r=r;
  camera_data_for_dtam_h->T_t=t;
  camera_data_for_dtam_h->cam_r_projected_on_cam_m=cam_r_projected_on_cam_m;
  camera_data_for_dtam_h->cam_r_depth_on_camera_m=cam_r_depth_on_camera_m;
  camera_data_for_dtam_h->cam_r_in_front=cam_r_in_front;
  query_proj_matrix_.create(rows,cols,CV_32FC3);
  camera_data_for_dtam_h->query_proj_matrix=query_proj_matrix_;



  // Kernel invocation
  dim3 threadsPerBlock( 8 , 8 , 1);
  dim3 numBlocks( rows/8, cols/8 , 1);
  prepareCameraForDtam_kernel<<<numBlocks,threadsPerBlock>>>( camera_r->camera_gpu_, camera_m->camera_gpu_, camera_data_for_dtam_h->query_proj_matrix);
  printCudaError("Kernel preparing camera for dtam "+camera_m->name_);

  cudaMalloc(&camera_data_for_dtam_, sizeof(cameraDataForDtam));
  printCudaError("cudaMalloc (dtam constr) "+camera_m->name_);

  cudaMemcpy(camera_data_for_dtam_, camera_data_for_dtam_h, sizeof(cameraDataForDtam), cudaMemcpyHostToDevice);
  printCudaError("cudaMemcpy (dtam constr) "+camera_m->name_);

  delete camera_data_for_dtam_h;

}


__global__ void UpdateCostVolume_kernel(Camera_gpu* camera_r, Camera_gpu* camera_m,
              cv::cuda::PtrStepSz<int2> cost_volume, cameraDataForDtam* camera_data_for_dtam_,
              float* invdepth_r_array, bool occl){


  int row = blockIdx.x * blockDim.x + threadIdx.x;
  int col = blockIdx.y * blockDim.y + threadIdx.y;
  int i = blockIdx.z * blockDim.z + threadIdx.z;

  int cols = blockDim.y*gridDim.y;

  // initializations
  Eigen::Vector2f uv_r;
  Eigen::Vector2i pixel_coords_r(col,row);
  camera_r->pixelCoords2uv(pixel_coords_r, uv_r);
  bool stop = false;

  uchar3 clr_r = camera_r->image_rgb_(row,col);
  float depth1_r=camera_r->lens_;
  float depth2_r=camera_r->max_depth_;
  float3 val = camera_data_for_dtam_->query_proj_matrix(row,col);
  if (val.z<0)
    stop = true;
  Eigen::Vector2f uv1_fixed = camera_data_for_dtam_->cam_r_projected_on_cam_m;
  Eigen::Vector2f uv2_fixed;
  uv2_fixed.x()=val.x;
  uv2_fixed.y()=val.y;
  float depth1_m_fixed = camera_data_for_dtam_->cam_r_depth_on_camera_m;
  float depth2_m_fixed = val.z;
  Eigen::Matrix3f r=camera_data_for_dtam_->T_r;
  Eigen::Vector3f t=camera_data_for_dtam_->T_t;
  float f = camera_m->lens_;
  float w=camera_m->width_;
  float h=camera_m->width_/camera_m->aspect_;

  Eigen::Vector2i pixel_current;

  if(!stop){

    float depth_r = 1.0/invdepth_r_array[i];

    float depth_m = depth_r*r(2,2)-t(2)-((depth_r*r(2,0)*(2*uv_r.x()-w))/(2*f))-((depth_r*r(2,1)*(-2*uv_r.y()+h))/(2*f));
    float ratio_invdepth_m = ((1.0/depth_m)-(1.0/depth1_m_fixed))/((1.0/depth2_m_fixed)-(1.0/depth1_m_fixed));

    Eigen::Vector2f uv_current;
    uv_current.x()=uv1_fixed.x()+ratio_invdepth_m*(uv2_fixed.x()-uv1_fixed.x()) ;
    uv_current.y()=uv1_fixed.y()+ratio_invdepth_m*(uv2_fixed.y()-uv1_fixed.y()) ;

    camera_m->uv2pixelCoords( uv_current, pixel_current);

    if(pixel_current.x()<0 || pixel_current.y()<0 || pixel_current.x()>=(camera_m->resolution_) || pixel_current.y()>=(int)((float)camera_m->resolution_/(float)camera_m->aspect_) )
      stop=true;
  }

  int col_ = cols*i+col;

  if (!stop){

    uchar3 clr_current = camera_m->image_rgb_(pixel_current.y(),pixel_current.x());

    // int cost_current=((clr_r.x-clr_current.x)*(clr_r.x-clr_current.x)+(clr_r.y-clr_current.y)*(clr_r.y-clr_current.y)+(clr_r.z-clr_current.z)*(clr_r.z-clr_current.z));
    int cost_current=(abs(clr_r.x-clr_current.x)+abs(clr_r.y-clr_current.y)+abs(clr_r.z-clr_current.z));

    if(occl){
      cost_current=min(cost_current, 20);
      // cost_current=max(cost_current, 3);
    }

    if (cost_current<20 || occl)
    {
        // cost_current=max(cost_current, 3);
        // cost_current=min(cost_current, 50);

        int2 cost_volume_val = cost_volume(row,col_);

        cost_volume_val.x = (cost_volume_val.x*cost_volume_val.y+cost_current)/(cost_volume_val.y+1);

        cost_volume_val.y++;

        cost_volume(row,col_) = cost_volume_val;
    }

  }

}


//
// __global__ void StudyCostVolumeMin_kernel(Camera_gpu* camera_r, Camera_gpu* camera_m,
//               cv::cuda::PtrStepSz<int2> cost_volume, cameraDataForDtam* camera_data_for_dtam_, float* invdepth_r_array,
//               int row, int col, cv::cuda::PtrStepSz<float> depth_groundtruth, cv::cuda::PtrStepSz<float> a){
//
//
//   int i = blockIdx.z * blockDim.z + threadIdx.z;
//
//   int cols = camera_r->resolution_;
//
//   // initializations
//   Eigen::Vector2f uv_r;
//   Eigen::Vector2i pixel_coords_r(col,row);
//   camera_r->pixelCoords2uv(pixel_coords_r, uv_r);
//
//   bool stop = false;
//
//   uchar3 clr_r = camera_r->image_rgb_(row,col);
//   float depth1_r=camera_r->lens_;
//   float depth2_r=camera_r->max_depth_;
//   float3 val = camera_data_for_dtam_->query_proj_matrix(row,col);
//   if (val.z<0)
//     stop = true;
//   Eigen::Vector2f uv1_fixed = camera_data_for_dtam_->cam_r_projected_on_cam_m;
//   Eigen::Vector2f uv2_fixed;
//   uv2_fixed.x()=val.x;
//   uv2_fixed.y()=val.y;
//   float depth1_m_fixed = camera_data_for_dtam_->cam_r_depth_on_camera_m;
//   float depth2_m_fixed = val.z;
//   Eigen::Matrix3f r=camera_data_for_dtam_->T_r;
//   Eigen::Vector3f t=camera_data_for_dtam_->T_t;
//   float f = camera_m->lens_;
//   float w=camera_m->width_;
//   float h=camera_m->width_/camera_m->aspect_;
//
//   Eigen::Vector2i pixel_current;
//
//   if(!stop){
//
//     float depth_r = 1.0/invdepth_r_array[i];
//
//     // Eigen::Vector3f p;
//     // float depth_m;
//     // camera_r->pointAtDepth(uv_r, depth_r, p);
//     // Eigen::Vector2f uv_current;
//     // bool query_in_front = camera_m->projectPoint(p, uv_current, depth_m);
//
//     float depth_m = depth_r*r(2,2)-t(2)-((depth_r*r(2,0)*(2*uv_r.x()-w))/(2*f))-((depth_r*r(2,1)*(-2*uv_r.y()+h))/(2*f));
//     float ratio_invdepth_m = ((1.0/depth_m)-(1.0/depth1_m_fixed))/((1.0/depth2_m_fixed)-(1.0/depth1_m_fixed));
//     // float ratio_invdepth_m = ((1.0/depth_m)-(1.0/depth1_m_fixed))/((1.0/depth2_m_fixed)-(1.0/depth1_m_fixed));
//
//     // printf("%f\n", depth_m );
//
//     Eigen::Vector2f uv_current;
//     uv_current.x()=uv1_fixed.x()+ratio_invdepth_m*(uv2_fixed.x()-uv1_fixed.x()) ;
//     uv_current.y()=uv1_fixed.y()+ratio_invdepth_m*(uv2_fixed.y()-uv1_fixed.y()) ;
//
//     camera_m->uv2pixelCoords( uv_current, pixel_current);
//
//     if(pixel_current.x()<0 || pixel_current.y()<0 || pixel_current.x()>=(camera_m->resolution_) || pixel_current.y()>=(int)((float)camera_m->resolution_/(float)camera_m->aspect_) )
//       stop=true;
//   }
//
//   int col_ = cols*i+col;
//
//   int cost_current;
//   uchar3 clr_current;
//
//   if (!stop){
//
//     clr_current = camera_m->image_rgb_(pixel_current.y(),pixel_current.x());
//
//     // int cost_current=((clr_r.x-clr_current.x)*(clr_r.x-clr_current.x)+(clr_r.y-clr_current.y)*(clr_r.y-clr_current.y)+(clr_r.z-clr_current.z)*(clr_r.z-clr_current.z));
//     cost_current=(abs(clr_r.x-clr_current.x)+abs(clr_r.y-clr_current.y)+abs(clr_r.z-clr_current.z));
//
//     int val =255-(cost_current/3);
//     uchar3 magenta = make_uchar3(255,0,255);
//     uchar3 magenta_ = make_uchar3(val,0,val);
//
//     camera_m->image_rgb_(pixel_current.y(),pixel_current.x())=magenta;
//
//   }
//
//   __syncthreads();
//
//   __shared__ int cost_array[NUM_INTERPOLATIONS];
//   __shared__ int indx_array[NUM_INTERPOLATIONS];
//
//   cost_array[i]=cost_volume(row,col_).x;
//   indx_array[i]=i;
//
//   // REDUCTION
//   // Iterate of log base 2 the block dimension
//   for (int s = 1; s < NUM_INTERPOLATIONS; s *= 2) {
//     // Reduce the threads performing work by half previous the previous
//     // iteration each cycle
//     if (i % (2 * s) == 0) {
//       int min_cost = min(cost_array[i + s], cost_array[i]);
//
//       if (cost_array[i] > min_cost){
//         indx_array[i] = indx_array[i+s];
//         cost_array[i] = min_cost ;
//       }
//     }
//     __syncthreads();
//   }
//   __syncthreads();
//   // printf("cost current at i=%i is: %i\n",i, cost_current);
//
//   if (i == indx_array[0]) {
//   // if (i == 57) {
//   // if (i == 1) {
//     uchar3 blue = make_uchar3(255,0,0);
//     camera_m->image_rgb_(pixel_current.y(),pixel_current.x())=blue;
//
//     // printf("camera_r->max_depth_: %f\n", camera_r->max_depth_);
//     // printf("depth1_m_fixed: %f\n", depth1_m_fixed);
//     // printf("depth2_m_fixed: %f\n", depth2_m_fixed);
//     printf("predicted depth from costvol: %f\n", 1.0/invdepth_r_array[indx_array[0]]);
//     printf("predicted depth from a: %f\n", 1.0/(a(row,col)*invdepth_r_array[0]));
//     printf("grondtruth depth: %f\n", (1.0/(depth_groundtruth(row,col)*2)));
//     // printf("grondtruth val: %f\n", depth_groundtruth(row,col));
//     printf("clr_r is: %i,%i,%i\n", clr_r.x ,clr_r.y ,clr_r.z);
//     printf("clr_current is: %i,%i,%i\n", clr_current.x ,clr_current.y ,clr_current.z);
//     // printf("stop flag is: %i\n", stop);
//     printf("cost current is: %i\n", cost_current);
//     printf("cost current TOT is: %i\n", cost_volume(row,col_).x);
//     printf("n projections: %i\n", cost_volume(row,col_).y);
//     // printf("cost current is: %i\n", cost_current);
//     printf("min cost is: %i\n", cost_array[0]);
//     // printf("coords: %i %i\n", pixel_current.y(), pixel_current.x());
//     printf("idx: %i\n", i);
//     printf("\n");
//   }
//
// }


__global__ void ComputeCostVolumeMin_kernel(cv::cuda::PtrStepSz<float> a, cv::cuda::PtrStepSz<int2> cost_volume, float* invdepth_r_array){

  int row = blockIdx.x * blockDim.x + threadIdx.x;
  int col = blockIdx.y * blockDim.y + threadIdx.y;
  int i = blockIdx.z * blockDim.z + threadIdx.z;

  int cols = blockDim.y*gridDim.y;
  int col_ = cols*i+col;


  __shared__ int cost_array[4][4][NUM_INTERPOLATIONS];
  __shared__ int indx_array[4][4][NUM_INTERPOLATIONS];

  cost_array[threadIdx.x][threadIdx.y][i]=cost_volume(row,col_).x;
  indx_array[threadIdx.x][threadIdx.y][i]=i;

  // REDUCTION
  // Iterate of log base 2 the block dimension
	for (int s = 1; s < NUM_INTERPOLATIONS; s *= 2) {
		// Reduce the threads performing work by half previous the previous
		// iteration each cycle
		if (i % (2 * s) == 0) {
      int min_cost = min(cost_array[threadIdx.x][threadIdx.y][i + s], cost_array[threadIdx.x][threadIdx.y][i]);
      if (cost_array[threadIdx.x][threadIdx.y][i] > min_cost){
        indx_array[threadIdx.x][threadIdx.y][i] = indx_array[threadIdx.x][threadIdx.y][i+s];
        cost_array[threadIdx.x][threadIdx.y][i] = min_cost ;
      }
		}
		__syncthreads();
	}
  if (i == indx_array[threadIdx.x][threadIdx.y][0]) {

    // printf("%i\n", cost_array[threadIdx.x][threadIdx.y][0] );

    float val =(float)invdepth_r_array[i]/invdepth_r_array[0];

    a(row,col)=val;
    if (indx_array[threadIdx.x][threadIdx.y][0]==0){
      a(row,col)=1.0;
    }
	}

}



void Mapper::UpdateCostVolume(int index_m, bool occl){
  double t_s=getTime();

  // Camera_cpu* camera_r_cpu = camera_vector_cpu_[index_r_];
  // Camera_gpu* camera_r_gpu = camera_r_cpu->camera_gpu_;
  // Camera_gpu* camera_m_gpu = camera_vector_cpu_[index_m]->camera_gpu_;
  //
  // int cols = camera_r_cpu->invdepth_map_->image_.cols;
  // int rows = camera_r_cpu->invdepth_map_->image_.rows;
  //
  // prepareCameraForDtam(index_m);
  //
  // dim3 threadsPerBlock( 4 , 4 , NUM_INTERPOLATIONS);
  // dim3 numBlocks( rows/4, cols/4 , 1);
  // UpdateCostVolume_kernel<<<numBlocks,threadsPerBlock>>>(camera_r_gpu, camera_m_gpu, cost_volume_, camera_data_for_dtam_, invdepth_r_array_, occl);
  // printCudaError("Kernel updating cost volume");
  //
  // double t_e=getTime();
  // double delta=t_e-t_s;
  //
  // std::cerr << "cost volume computation took: " << delta << " ms " << std::endl;


}

double Mapper::StudyCostVolumeMin(int index_m, int row, int col,bool showbaseline=false){
  // double t_s=getTime();

  //
  // Camera_cpu* camera_r_cpu = camera_vector_cpu_[index_r_];
  // Camera_gpu* camera_r_gpu = camera_r_cpu->camera_gpu_;
  // Camera_gpu* camera_m_gpu = camera_vector_cpu_[index_m]->camera_gpu_;
  //
  // dim3 threadsPerBlock( 1 , 1 , NUM_INTERPOLATIONS);
  // dim3 numBlocks( 1, 1 , 1);
  // StudyCostVolumeMin_kernel<<<numBlocks,threadsPerBlock>>>(camera_r_gpu, camera_m_gpu, cost_volume_, camera_data_for_dtam_, invdepth_r_array_,row,col, depth_groundtruth_, a);
  // printCudaError("Kernel studying cost volume");
  //
  // if(showbaseline){
  //   Image< cv::Vec3b >* study_baseline = new Image< cv::Vec3b >("Study baseline");
  //   camera_vector_cpu_[index_m]->image_rgb_gpu_.download(study_baseline->image_);
  //   study_baseline->show(1500/camera_vector_cpu_[index_m]->resolution_);
  //   delete study_baseline;
  //
  // }
  //
  //
  //
  // if(showbaseline){
  //   Image< cv::Vec3b >* study_ref = new Image< cv::Vec3b >("Study reference");
  //   camera_vector_cpu_[index_r_]->image_rgb_gpu_.download(study_ref->image_);
  //   Eigen::Vector2i pxl(col,row);
  //   cv::Vec3b red(0,0,255);
  //   study_ref->setPixel(pxl, red);
  //   study_ref->show(1500/camera_vector_cpu_[index_r_]->resolution_);
  // }
  //
  // double t_e=getTime();
  // double delta=t_e-t_s;
  // return delta;
}

void Mapper::ComputeCostVolumeMin(){

  // double t_s=getTime();
  //
  // Camera_cpu* camera_r_cpu = camera_vector_cpu_[index_r_];
  // Camera_gpu* camera_r_gpu = camera_r_cpu->camera_gpu_;
  // int cols = camera_r_cpu->invdepth_map_->image_.cols;
  // int rows = camera_r_cpu->invdepth_map_->image_.rows;
  //
  // dim3 threadsPerBlock( 4 , 4 , NUM_INTERPOLATIONS);
  // dim3 numBlocks( rows/4, cols/4 , 1);
  // ComputeCostVolumeMin_kernel<<<numBlocks,threadsPerBlock>>>( camera_r_cpu->invdepth_map_gpu_, cost_volume_, invdepth_r_array_);
  // printCudaError("Kernel computing cost volume min");
  //
  // double t_e=getTime();
  // double delta=t_e-t_s;
  // std::cerr << "ComputeCostVolumeMin took: " << delta << " ms " << std::endl;
}




__global__ void maxReduction_kernel(float *v, float *v_r, int size) {
	// Allocate shared memory
	__shared__ float partial_max[MAX_THREADS];

	// Calculate thread ID
	int tid = blockIdx.x * blockDim.x + threadIdx.x;

	// Load elements into shared memory
  if (tid<size)
  	partial_max[threadIdx.x] = v[tid];
  else
    partial_max[threadIdx.x] = 0;

	__syncthreads();

	// Iterate of log base 2 the block dimension
	for (int s = 1; s < blockDim.x; s *= 2) {
		// Reduce the threads performing work by half previous the previous
		// iteration each cycle
		if (threadIdx.x % (2 * s) == 0) {
			partial_max[threadIdx.x] = fmaxf(abs(partial_max[threadIdx.x + s]),abs(partial_max[threadIdx.x]));
		}
		__syncthreads();
	}

	// Let the thread 0 for this block write it's result to main memory
	// Result is inexed by this block
	if (threadIdx.x == 0) {
		v_r[blockIdx.x] = partial_max[0];
	}
}


void Mapper::StateFromGt(){
  Camera_cpu* camera = camera_vector_cpu_[index_r_];
  depth_groundtruth_.download(camera->invdepth_map_->image_);
  camera->getCamera_gpu();
}

__global__ void depthSamplingInit_kernel( cv::cuda::PtrStepSz<float> invdepth_r_volume, float* invdepth_r_array){
  int row = blockIdx.x * blockDim.x + threadIdx.x;
  int col = blockIdx.y * blockDim.y + threadIdx.y;
  int i = blockIdx.z * blockDim.z + threadIdx.z;

  int cols = blockDim.y*gridDim.y;
  int col_ = cols*i+col;
  
  invdepth_r_volume(row,col_)=invdepth_r_array[i];
}

void Mapper::depthSamplingInit(Environment_gpu* environment){

  int rows = environment->resolution_/environment->aspect_;
  int cols = environment->resolution_;
  float depth1_r=environment->min_depth_;
  float depth2_r=environment->max_depth_;

  float* invdepth_r_array_h = new float[NUM_INTERPOLATIONS];
  float* invdepth_r_array_d = new float[NUM_INTERPOLATIONS];

  switch_idx_=40;
  switch_depth_=2;
  for (int i=0; i<switch_idx_; i++){
    float ratio_depth_r = (float)(i)/((float)switch_idx_);
    float depth_r = depth1_r+ratio_depth_r*(switch_depth_-depth1_r);
    invdepth_r_array_h[i]=1.0/depth_r;
    std::cout << "depth: " << depth_r  << ", idx: " << i << std::endl;
  }
  for (int i=switch_idx_; i<NUM_INTERPOLATIONS; i++){
    float ratio_depth_r = (float)(i-switch_idx_)/((float)NUM_INTERPOLATIONS-switch_idx_-1);
    float invdepth_r = (1.0/switch_depth_)+ratio_depth_r*((1.0/depth2_r)-(1.0/switch_depth_));
    float depth_r = 1.0/invdepth_r;
    invdepth_r_array_h[i]=1.0/depth_r;
    std::cout << "depth: " << depth_r  << ", idx: " << i << std::endl;
  }

  cudaError_t err ;

  cudaMalloc(&invdepth_r_array_d, sizeof(float)*NUM_INTERPOLATIONS);
  err = cudaGetLastError();
  if (err != cudaSuccess)
      printf("cudaMalloc (dtam constr) Error: %s\n", cudaGetErrorString(err));

  cudaMemcpy(invdepth_r_array_d, invdepth_r_array_h, sizeof(float)*NUM_INTERPOLATIONS, cudaMemcpyHostToDevice);
  err = cudaGetLastError();
  if (err != cudaSuccess)
      printf("cudaMemcpy (dtam constr) Error: %s\n", cudaGetErrorString(err));

  delete (invdepth_r_array_h);

  invdepth_r_volume_.create(rows,cols,CV_32FC1);

  // Kernel invocation
  dim3 threadsPerBlock( 4 , 4 , NUM_INTERPOLATIONS);
  dim3 numBlocks( rows/4, cols/4 , 1);
  depthSamplingInit_kernel<<<numBlocks,threadsPerBlock>>>( invdepth_r_volume_, invdepth_r_array_d );
  printCudaError("depthSamplingInit ");

}
