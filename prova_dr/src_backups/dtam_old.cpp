#include "dtam.h"
#include <math.h>


bool Dtam::get1stDepthWithUV(Camera* camera_r, Camera* camera_m, Eigen::Vector2f& uv_r, Eigen::Vector2f& uv_m, float& depth){

  Eigen::Isometry3f T = camera_m->frame_world_wrt_camera_*camera_r->frame_camera_wrt_world_;
  auto r=T.linear();
  auto t=T.translation();
  float f = camera_r->lens_;
  float w=camera_m->width_;
  float h=camera_m->width_/camera_m->aspect_;
  depth = (2*f*(f+t(2)))/(2*f*r(2,2)-2*r(2,0)*uv_r.x()+r(2,0)*w-r(2,1)*(h-2*uv_r.y()));
  uv_m.x() = t(0)+(w/2)-depth*r(0,2)+((depth*r(0,0)*(2*uv_r.x()-w))/(2*f))+((depth*r(0,1)*(h-2*uv_r.y()))/(2*f));
  uv_m.y() = (h/2)-t(1)+depth*r(1,2)-((depth*r(1,0)*(2*uv_r.x()-w))/(2*f))-((depth*r(1,1)*(h-2*uv_r.y()))/(2*f));

}

int Dtam::closest(std::vector<float>& vec, float value) {

  // std::vector<int>::iterator upper1;
  std::vector<float>::iterator it_up = std::upper_bound(vec.begin(), vec.end(), value);
  // auto const it_up = std::upper_bound(vec.begin(), vec.end(), value);
  int idx_up = (it_up - vec.begin());
  int idx_low = (idx_up-1);
  float dist_low = value-vec[idx_low];
  float dist_up= vec[idx_up]-value;
  if (idx_up<=0){
    return -1;
  }
  if (idx_up >= vec.size())
    return -1;

  if (dist_low<dist_up){
    return idx_low;
  }
  else{
    return idx_up;
  }

}


bool Dtam::getTotalCosts(std::vector<std::vector<std::tuple<float,int>>>& allCosts, std::vector<std::tuple<float,int>>& totalCosts)
{
  if(allCosts.size()==1){
    if (!allCosts[0].empty())
      totalCosts=allCosts[0];
    else
      return false;
    // for (auto cost : totalCosts)
    //   std::cout << std::get<0>(cost) << std::endl;
    // std::cout << "" << std::endl;
  }
  else{
    std::vector<std::vector<float>> depthsVec;
    for (int i=0; i<allCosts.size(); i++){

      // for (auto cost : allCosts[i])
      //   std::cout << std::get<0>(cost) << " " << std::get<1>(cost) << std::endl;
      // std::cout << "" << std::endl;
      std::vector<float> depths;
      if (allCosts[i].size()==0){
        depths.clear();
        depthsVec.push_back(depths);
      }
      else{
        extractDepthsFromTupleVec( allCosts[i], depths);
        depthsVec.push_back(depths);
      }
    }

    for (int i=0; i<allCosts.size(); i++){
      if (allCosts[i].empty()){
        continue;
      }
      for (int j=0; j<allCosts[i].size(); j++){

        int cost = std::get<1>(allCosts[i][j]);
        float depth = std::get<0>(allCosts[i][j]);
        for (int k=0; k<allCosts.size(); k++){

          if (i!=k && !allCosts[k].empty()){
            int idx = closest(depthsVec[k], depth);
            if(idx!=-1){
              int cost_delta = std::get<1>(allCosts[k][idx]);
              // if (idx>(allCosts[k].size()-1))
              //   std::cout << idx << " " << allCosts[k].size() << std::endl;
              cost += cost_delta;
              // if(idx>depthsVec[k].size()-1)
              // if(idx==-1)
              //   std::cout << "vec size: " << depthsVec[k].size()-1 << ", idx: " << idx << std::endl;
            }
            else
              cost = 10000000;

          }

        }
        std::tuple<float,int> tuple = std::tuple<float,int>(depth,cost);

        totalCosts.push_back(tuple);
        // std::cout << depth << std::endl;
        // std::cout << cost << "\n"<< std::endl;
        // std::cout << totalCosts.size() << std::endl;

      }
    }
  }
  return true;
}

void Dtam::extractDepthsFromTupleVec(std::vector<std::tuple<float,int>>& totalCosts, std::vector<float>& depths){
    depths.resize(totalCosts.size());
    for (int i = 0; i < totalCosts.size(); i++) {
        depths[i] = std::get<0>(totalCosts[i]);
    }
}

void Dtam::extractCostsFromTupleVec(std::vector<std::tuple<float,int>>& totalCosts, std::vector<float>& costs){
    costs.resize(totalCosts.size());
    for (int i = 0; i < totalCosts.size(); i++) {
        costs[i] = std::get<1>(totalCosts[i]);
    }
}



int Dtam::getIndexOfMinimumCost(std::vector<std::tuple<float,int>>& totalCosts){
  std::vector<float> costs;
  extractCostsFromTupleVec( totalCosts, costs);
  if (costs.size()==0)
    return -1;
  // std::cout << invdepths.size() << std::endl;
  int index = std::min_element(costs.begin(),costs.end()) - costs.begin();
  return index;
  // float invdepth = totalCosts[index].first;
  // float depth = 1.0/invdepth;
  // return depth;

}

// bool Dtam::collectCostsInEpipolarLine_FVT(Eigen::Vector2i& pixel_coords_r, Camera* camera_r, Camera* camera_m,
//    std::vector<std::tuple<float,int>>& costInvdepthTuple){
//   Eigen::Vector3f camera_r_p = camera_r->frame_camera_wrt_world_.translation();
//   Eigen::Vector2f cam_r_projected_on_cam_m;
//   float cam_r_depth_on_camera_m;
//   bool cam_r_in_front = camera_m->projectPoint(camera_r_p, cam_r_projected_on_cam_m, cam_r_depth_on_camera_m);
//
//   Eigen::Vector3f query_p = camera_r->frame_camera_wrt_world_.translation();
//   Eigen::Vector2f uv_r;
//   camera_r->pixelCoords2uv(pixel_coords_r, uv_r);
//   camera_r->pointAtDepth(uv_r, camera_r->max_depth_, query_p);
//   // camera_r->pointAtDepth(uv_r, 0.3, query_p);
//   Eigen::Vector2f query_p_projected_on_cam_m;
//   float query_depth_on_camera_m;
//   bool query_in_front = camera_m->projectPoint(query_p, query_p_projected_on_cam_m, query_depth_on_camera_m);
//
//   Eigen::Vector2f uv1;
//   Eigen::Vector2f uv2;
//   float depth1;
//   float depth2;
//
//   // if both camera r and query point are on back of camera m return false
//   if (!query_in_front && !cam_r_in_front)
//     return false;
//   // if query point is in front of camera m whereas camera r is on the back
//   else if (query_in_front && !cam_r_in_front){
//     // std::cout << "query in front" << std::endl;
//
//     uv2=query_p_projected_on_cam_m;
//     depth2=query_depth_on_camera_m;
//     Dtam::get1stDepthWithUV(camera_r, camera_m, uv_r, uv1, depth1);
//     camera_m->resizeLine(uv1 , uv2, depth1, depth2);
//
//   }
//   // if camera r is in front of camera m whereas query point is on the back
//   else if (!query_in_front && cam_r_in_front){
//     // TODO
//     return false;
//   }
//   // if both camera r and query point are in front of camera m
//   else {
//     // std::cout << "both in front" << std::endl;
//     uv1=cam_r_projected_on_cam_m;
//     uv2=query_p_projected_on_cam_m;
//     depth1=cam_r_depth_on_camera_m;
//     depth2=query_depth_on_camera_m;
//     camera_m->resizeLine(uv1 , uv2, depth1, depth2);
//   }
//
//   // Eigen::Vector3f p;
//   // camera_m->pointAtDepth(uv1, depth1, p);
//   // Eigen::Vector2f uv;
//   // float p_cam_z;
//   // camera_r->projectPoint(p, uv, p_cam_z );
//   // Eigen::Vector2i pixel_coords;
//   // camera_r->uv2pixelCoords( uv, pixel_coords);
//   // std::cout << depth2 << std::endl;
//   // std::cout << uv << std::endl;
//   // std::cout << pixel_coords << std::endl;
//
//   // Fast Voxel Traversal Algorithm
//
//   Eigen::Vector2i pixel_coords_1;
//   Eigen::Vector2i pixel_coords_2;
//   camera_m->uv2pixelCoords(uv1 , pixel_coords_1);
//   camera_m->uv2pixelCoords(uv2 , pixel_coords_2);
//   float pixel_width= camera_m->width_/camera_m->resolution_;
//   float steepness=(uv2.y()-uv1.y())/(uv2.x()-uv1.x());
//
//
//   int sign_x;
//   int sign_y;
//   int sign_steepness;
//
//   if (steepness>0)
//     sign_steepness=1;
//   else
//     sign_steepness=-1;
//
//   if ((uv2.x()-uv1.x())>0)
//     sign_x = 1;
//   else
//     sign_x = -1;
//   if ((uv2.y()-uv1.y())>0)
//     sign_y = 1;
//   else
//     sign_y = -1;
//
//   float delta_x = ((pixel_coords_1.x()+1)*pixel_width)-uv1.x();
//   float delta_y = ((pixel_coords_1.y()+1)*pixel_width)-uv1.y();
//
//   Eigen::Vector2i current_pixel = pixel_coords_1;
//
//   float tMaxX = (steepness)*delta_x;//+(pixel_coords_1.y()*pixel_width);
//   float tMaxY = (1.0/steepness)*delta_y;//+(pixel_coords_1.x()*pixel_width);
//
//   bool horizontal = steepness==0;
//
//
//   float tDeltaX = sign_y*sqrt(pow((steepness)*pixel_width,2)+pow(pixel_width,2));
//   float tDeltaY = sign_y*sqrt(pow((1.0/steepness)*pixel_width,2)+pow(pixel_width,2));
//
//   cv::Vec3b clr_r;
//   camera_r->image_rgb_->evalPixel(pixel_coords_r,clr_r);
//
//   int cost_min=999999; int cost_max=0;
//   int num_pixels = 0;
//   Eigen::Vector2i pixel_min; Eigen::Vector2i pixel_max;
//
//   while (sign_x*current_pixel.x()<=sign_x*pixel_coords_2.x() && sign_y*current_pixel.y()<=sign_y*pixel_coords_2.y()){
//   // for (int i=0; i<100; i++){
//     // std::cout << sign_y*current_pixel.y() << std::endl;
//     // std::cout << sign_y*pixel_coords_2.y() << std::endl;
//     // DO STUFF
//     cv::Vec3b clr_current;
//     camera_m->image_rgb_->evalPixel(current_pixel,clr_current);
//     int cost_current = mseBetween2Colors(clr_r, clr_current);
//     num_pixels++;
//     std::tuple<float,int> tuple(0.0,cost_current);
//     costInvdepthTuple.push_back(tuple);
//
//     if (cost_current<cost_min){
//       cost_min=cost_current;
//       pixel_min=current_pixel;
//     }
//     // if (cost_current>cost_max){
//     //   cost_max=cost_current;
//     //   pixel_max=current_pixel;
//     // }
//
//
//     if(sign_y*tMaxX<sign_y*tMaxY || horizontal){
//       tMaxX+=tDeltaX;
//       current_pixel.x()+=sign_x;
//     }
//     else{
//       tMaxY+=tDeltaY;
//       current_pixel.y()+=sign_y;
//     }
//
//     // // show epipolar line
//     // cv::Vec3b clr(255,0,255);
//     // camera_m->image_rgb_->setPixel(current_pixel, clr);
//   }
//   // Eigen::Vector2i pxl1;
//   // camera_m->uv2pixelCoords( uv1, pxl1);
//   // Eigen::Vector2i pxl2;
//   // camera_m->uv2pixelCoords( uv2, pxl2);
//   // cv::Vec3b clr1(0,255,255);
//   // cv::Vec3b clr2(0,255,0);
//   // camera_m->image_rgb_->setPixel(pxl1, clr1);
//   // camera_m->image_rgb_->setPixel(pxl2, clr2);
//   // cv::Vec3b clr(0,0,255);
//   // camera_m->image_rgb_->setPixel(pixel_min, clr);
//
//   float invdepth_delta=(1.0/depth1)-(1.0/depth2);
//   for (int i=0; i<costInvdepthTuple.size(); i++){
//     float ratio = (float)i/(float)(num_pixels);
//     float invdepth_ = ratio*invdepth_delta;
//     float invdepth = ((1.0/depth1)-invdepth_);
//     std::get<0>(costInvdepthTuple[i]) = invdepth;
//   }
//
//   // camera_m_->projectPixel(cp);
//
//   return true;
// }

bool Dtam::collectCostsInEpipolarLine(Eigen::Vector2i& pixel_coords_r, Camera* camera_r, Camera* camera_m,
   std::vector<std::tuple<float,int>>& costInvdepthTuple){
  Eigen::Vector3f camera_r_p = camera_r->frame_camera_wrt_world_.translation();
  Eigen::Vector2f cam_r_projected_on_cam_m;
  float cam_r_depth_on_camera_m;
  bool cam_r_in_front = camera_m->projectPoint(camera_r_p, cam_r_projected_on_cam_m, cam_r_depth_on_camera_m);

  Eigen::Vector3f query_p = camera_r->frame_camera_wrt_world_.translation();
  Eigen::Vector2f uv_r;
  camera_r->pixelCoords2uv(pixel_coords_r, uv_r);
  camera_r->pointAtDepth(uv_r, camera_r->max_depth_, query_p);
  // camera_r->pointAtDepth(uv_r, 0.3, query_p);
  Eigen::Vector2f query_p_projected_on_cam_m;
  float query_depth_on_camera_m;
  bool query_in_front = camera_m->projectPoint(query_p, query_p_projected_on_cam_m, query_depth_on_camera_m);

  Eigen::Vector2f uv1;
  Eigen::Vector2f uv2;
  float depth1;
  float depth2;

  // if both camera r and query point are on back of camera m return false
  if (!query_in_front && !cam_r_in_front)
    return false;
  // if query point is in front of camera m whereas camera r is on the back
  else if (query_in_front && !cam_r_in_front){
    // std::cout << "query in front" << std::endl;

    uv2=query_p_projected_on_cam_m;
    depth2=query_depth_on_camera_m;
    Dtam::get1stDepthWithUV(camera_r, camera_m, uv_r, uv1, depth1);
    depth1=camera_r->lens_;
    bool flag = camera_m->resizeLine(uv1 , uv2, depth1, depth2);
    if (!flag)
      return false;

  }
  // if camera r is in front of camera m whereas query point is on the back
  else if (!query_in_front && cam_r_in_front){
    // TODO
    return false;
  }
  // if both camera r and query point are in front of camera m
  else {
    // std::cout << "both in front" << std::endl;

    uv1=cam_r_projected_on_cam_m;
    uv2=query_p_projected_on_cam_m;
    depth1=cam_r_depth_on_camera_m;
    depth2=query_depth_on_camera_m;
    bool flag = camera_m->resizeLine(uv1 , uv2, depth1, depth2);
    if (!flag)
      return false;


    // Eigen::Vector3f p;
    // camera_r->pointAtDepth(uv_r, 0.5, p);
    // float p_cam_z;
    // camera_m->projectPoint(p, uv1, p_cam_z );
    // depth1=0.5;
    // uv2=query_p_projected_on_cam_m;
    // depth2=camera_r->max_depth_;
    // camera_m->resizeLine(uv1 , uv2, depth1, depth2);
  }



  cv::Vec3b clr_r;
  camera_r->image_rgb_->evalPixel(pixel_coords_r,clr_r);
  float pixel_width= camera_m->width_/camera_m->resolution_;
  float delta_u=uv2.x()-uv1.x();
  float delta_v=uv2.y()-uv1.y();
  int num_interpolations = (abs(delta_v)+abs(delta_u))/(pixel_width);
  if (num_interpolations==1)
    return false;
  float invdepth_delta=(1.0/depth1)-(1.0/depth2);
  // std::cout << "depth2: " << depth2 << std::endl;

  Eigen::Isometry3f T = camera_r->frame_world_wrt_camera_*camera_m->frame_camera_wrt_world_;
  auto r=T.linear();
  auto t=T.translation();
  float f = camera_r->lens_;
  float w=camera_m->width_;
  float h=camera_m->width_/camera_m->aspect_;

  for (int i=0; i<num_interpolations; i++){
  // for (int i=0; i<5; i++){
    float ratio = (float)i/((float)num_interpolations-1);
    Eigen::Vector2f uv_current;
    uv_current.x()=uv1.x()+ratio*delta_u;
    uv_current.y()=uv1.y()+ratio*delta_v;
    Eigen::Vector2i pixel_current;
    camera_m->uv2pixelCoords( uv_current, pixel_current);
    cv::Vec3b clr_current;
    camera_m->image_rgb_->evalPixel(pixel_current,clr_current);
    int cost_current = mseBetween2Colors(clr_r, clr_current);
    float invdepth_m_ = ratio*invdepth_delta;
    float invdepth_m = ((1.0/depth1)-invdepth_m_);
    float depth_m = 1.0/invdepth_m;
    float depth_r = depth_m*r(2,2)-t(2)-((depth_m*r(2,0)*(2*uv_current.x()-w))/(2*f))-((depth_m*r(2,1)*(-2*uv_current.y()+h))/(2*f));
    float invdepth_r = 1.0/depth_r;
    // float invdepth_r = 1.0/depth_r;
    // std::cout << "invdepth: " << invdepth << ", cost_current: " << cost_current << std::endl;


    // std::cout << "i: " << i << ", cost: " << cost_current << ", clr_r: " << clr_r << ", clr_current: " << clr_current << ", depth: " << 1.0/invdepth << std::endl;


    // Eigen::Vector3f p;
    // camera_m->pointAtDepth(uv_current, 1.0/invdepth, p);
    // Eigen::Vector2f uv_m;
    // float p_cam_z;
    // camera_r->projectPoint(p, uv_m, p_cam_z );
    // Eigen::Vector2i pixel_coords_m;
    // camera_m->uv2pixelCoords( uv_m, pixel_coords_m);
    // if (pixel_coords_m != pixel_coords_r)
    //   std::cout << pixel_coords_m << std::endl;

    std::tuple<float,int> tuple(invdepth_r,cost_current);
    costInvdepthTuple.push_back(tuple);

    // // show epipolar line
    // cv::Vec3b clr(255,0,255);
    // camera_m->image_rgb_->setPixel(pixel_current, clr);
    // std::cout << ", cost: " << cost_current << ", invdepth: " << invdepth << std::endl;

  }
  // int index = getIndexOfMinimumCost(costInvdepthTuple);
  // cv::Vec3b clr(0,0,255);
  // Eigen::Vector2f uv_ = std::get<2>(costInvdepthTuple[index]);
  // Eigen::Vector2i pxl;
  // camera_m->uv2pixelCoords( uv_, pxl);
  // camera_m->image_rgb_->setPixel(pxl, clr);


  return true;
}


void Dtam::getDepthMap(CameraVector& camera_vector){
  int cols = camera_vector[0]->depth_map_->image_.cols;
  int rows = camera_vector[0]->depth_map_->image_.rows;
  for (int row = 0; row<rows; row++)
    for (int col = 0; col<cols; col++){
      std::vector<std::vector<std::tuple<float,int>>> allCosts;
      std::vector<std::tuple<float,int>> totalCosts;

      Eigen::Vector2i pixel_coords_r(col,row);

      // collectCostsInEpipolarLine_FVT(pixel_coords_r, camera_vector[0], camera_vector[1], costInvdepthTuple);
      if (camera_vector.size()<2)
        continue;
      else if (camera_vector.size()==2){
        std::vector<std::tuple<float,int>> costInvdepthTuple;
        bool flag = collectCostsInEpipolarLine(pixel_coords_r, camera_vector[0], camera_vector[1], costInvdepthTuple);
        if (!flag || costInvdepthTuple.size()==0)
          continue;

        allCosts.push_back(costInvdepthTuple);
      }
      else{
        for (int i=1; i<camera_vector.size(); i++){
          std::vector<std::tuple<float,int>> costInvdepthTuple;
          if (i!=0){
            bool flag = collectCostsInEpipolarLine(pixel_coords_r, camera_vector[0], camera_vector[i], costInvdepthTuple);
            if (!flag || costInvdepthTuple.size()==0){
              costInvdepthTuple.clear();
            }

            allCosts.push_back(costInvdepthTuple);
          }
        }

      }

      getTotalCosts(allCosts, totalCosts);
      // for (int i=0; i<totalCosts.size(); i++){
        // std::cout << std::get<0>(totalCosts[i]) << std::endl;
        // if (std::get<0>(totalCosts[i])<=0.2)
        //   std::cout << std::get<0>(totalCosts[i]) << std::endl;
      // if (totalCosts.size()<=1)
      //   std::cout << totalCosts.size() << std::endl;
      // }
      int index = getIndexOfMinimumCost(totalCosts);
      if (index==-1)
        continue;
      // std::cout << index << std::endl;
      float invdepth_r = std::get<0>(totalCosts[index]);
      int cost = std::get<1>(totalCosts[index]);
      // if (index>totalCosts.size()-1)
      // std::cout << totalCosts.size() << std::endl;
      //
      //
      if(cost<9999100){
        float depth_value = (1.0/invdepth_r)/camera_vector[0]->max_depth_;
        camera_vector[0]->depth_map_->setPixel(pixel_coords_r,depth_value);
      }

    }
  // Eigen::Vector2i pixel_coords_r(50,50);
  // std::vector<std::tuple<float,int>> costInvdepthTuple;
  // dtam->collectCostsInEpipolarLine_FVT(pixel_coords_r, camera_r, camera_m, costInvdepthTuple);
  // std::vector<std::vector<std::tuple<float,int>>> allCosts;
  // std::vector<std::tuple<float,int>> totalCosts;
  // allCosts.push_back(costInvdepthTuple);
  // dtam->getTotalCosts(allCosts, totalCosts);
  // dtam->getDepthOfMinimumCost(totalCosts);
}
