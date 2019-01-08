#include "ekf_slam/system.h"
namespace ekf_slam {
System::System(LineDetector::Ptr line_detector, EKFilter::Ptr ekfilter)
:state_(State::INITIALIZING),line_detector_(line_detector),ekfilter_(ekfilter),frame_prev_(nullptr),map_(nullptr)
{}
void System::initSLAM(const Vector3d& T_init, LaserFrame::Ptr frame_init, const Matrix3d& P_T_init, const Vector2d& joint_value_init)
{
  //set initial pose
  T_prev_ = T_init;
  //set initial joint state
  joint_value_prev_ = joint_value_init;
  //detect all the lines in the first frame 
  vector<LineSegment> line_segments;
  line_detector_->detect(frame_init->laser_points_,line_segments);
  frame_prev_ = frame_init;
  //make a initial covariance matrix of map 
  Matrix2d P_ls_init = Matrix2d::Zero();
  P_ls_init(0,0) = 0.1;
  P_ls_init(1,1) = 0.1;
  vector<Matrix2d> P_line_segments(line_segments.size(),P_ls_init);
  // create a map and add all deteced line_segments to it 
  map_ = std::make_shared<Map>(line_segments,P_line_segments);
  //build covariance matrix 
  int map_size = map_->size();
  int dim = 3+2*map_size;
  P_prev_ = MatrixXd::Zero(dim,dim);
  P_prev_.block(0,0,3,3) = P_T_init;
  P_prev_.block(3,3,2*map_size,2*map_size) = map_->getCovarianceMatrix();
}
void System::oneSLAMStep(LaserFrame::Ptr frame_new, const Vector2d& joint_value_new, const int& wheel_radius)
{
  //compute control input 
  Vector2d u = (joint_value_new - joint_value_prev_)*wheel_radius;
  joint_value_prev_ = joint_value_new;
  //get observation 
  vector<LineSegment> z;
  line_detector_->detect(frame_new->laser_points_,z);
  frame_prev_ = frame_new;//may be don't need this
  //get observation covariance matrix 
  //just use fixed covariance, can be changed to be dependent on detection in future
  Matrix2d r = Matrix2d::Zero();
  r(0,0) = 0.1; r(1,1) = 0.1;
  vector<Matrix2d> R(z.size(),r);
  //posterior estimation
  Vector3d T_posterior;
  MatrixXd P_posterior;
  vector<LineSegment> line_segments_posterior;
  
  //extended kalman filter step 
  ekfilter_->ekfOneStep(T_prev_,u,z,R,map_->getMap(),P_prev_,T_posterior,line_segments_posterior,P_posterior);
  
  //Update Mapping 
  //update existing lines
  vector<Matrix2d> P_line_segments(map_->size());
  for(int i=0; i<map_->size();i++){
    P_line_segments[i] = (P_posterior.block(3+2*i,3+2*i,2,2));
  }
  map_->updateExistingLineSegments(line_segments_posterior,P_line_segments);
  //get unmatched LineSegment
  vector<int> unmatched_idx = ekfilter_->unmatched_idx;
  //convert LineSegment in laser frame to World frame
  Vector3d T_l_w;
  T_l_w(0) = -T_posterior(0) * cos(T_posterior(2)) - T_posterior(1) * sin(T_posterior(2));
  T_l_w(1) =  T_posterior(0) * sin(T_posterior(2)) - T_posterior(1) * cos(T_posterior(2));
  T_l_w(2) = -T_posterior(2);
  //for every unmatched observation 
  for(int i=0; i<unmatched_idx.size(); i++){
    //convert unmatched observation from laser frame to world frame using the postrior estimation 
    z[unmatched_idx[i]].convertToFrameT(T_l_w);
    //add to the map, the covariance matrix just use r,
    //can be modified to take state's covariance into account 
    map_->addOneLineSegment(z[unmatched_idx[i]],r);
  }
  //iterate, pass posterior estimation to prev 
  T_prev_ = T_posterior;
  //need to expand the covariance matrix if added new line to the map
  int n = unmatched_idx.size();
  if(n!=0){
    MatrixXd P = MatrixXd::Zero(3+2*map_->size(),3+2*map_->size());
    P.block(0,0,3,3) = P_posterior.block(0,0,3,3);
    P.block(3,3,2*map_->size(),2*map_->size()) = map_->getCovarianceMatrix();
    P_prev_ = P;
  } else{
    //assume no map element is deleted
    P_prev_ = P_posterior;
  }
}

}