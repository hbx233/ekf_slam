#include "ekf_slam/ekf.h"
#include "ekf_slam/common.h"
#include "ekf_slam/line_segment.h"
#include "ekf_slam/laser_frame.h"
#include <iomanip>
using ekf_slam::EKFilter;
using ekf_slam::LaserFrame;
using ekf_slam::LineSegment;

int main(){
  EKFilter ekf(0.05,0.1,1);
  Vector3d x_prev;
  Matrix3d P_prev;
  x_prev << 0.333542078036589,3.75724002564630,-0.671681351510224;
  P_prev << 0.1,0,0,0,0.1,0,0,0,0.1;
  vector<double> alpha{0.0063, 0.6537,-1.2716,-0.9752,-1.1639,-1.0172,2.3454,1.0894};
  vector<double> r{0.3467,-2.5259,0.7271,1.6774,-2.8934,0.3542,1.1883,0.3709};
  vector<LineSegment> landmarks;
  for(int i=0; i<alpha.size();i++){
    landmarks.push_back(LineSegment(alpha[i],r[i]));
  }
  //z
  vector<LineSegment> z;
  alpha = vector<double>{-2.5224,-1.7187,-0.5708,-0.4975,-0.6633,-0.1059,-0.2678,-1.4506};
  r = vector<double>{0.0380,5.2591,4.3229,4.7227,0.2601,3.4013,1.2589,2.9059};
  for(int i=0; i<alpha.size();i++){
    z.push_back(LineSegment(alpha[i],r[i]));
  }
  //R
  vector<Matrix2d> R;
  for(int i=0; i<alpha.size(); i++){
    R.push_back(Matrix2d::Identity()/10);
  }
  //u
  Vector2d u;
  u << 0.00189280253910503,0.00919748352913642;
  Vector3d x_posterior;
  Matrix3d P_posterior;
  ekf.ekfOneStep(x_prev,P_prev,u,z,R,landmarks,x_posterior,P_posterior);
  cout<<"Posterior state estimation"<<endl;
  cout<<x_posterior<<endl;
  cout<<"Posterior covariance estimation"<<endl;
  cout<<P_posterior<<endl;
#if 0
  //1. test state transition 
  //construct test data 
  Vector3d x_prev(3.14723686393179,4.05791937075619,-2.34371095524892);
  Matrix3d P_prev;
  P_prev << 0.1,0,0,0,0.1,0,0,0,0.1;
  Vector2d u(0.00826751712278039,0.00264718492450819);
  Vector3d x_hat;
  Matrix3d P_hat;
  Matrix3d F_x;
  Matrix32d F_u;
  ekf.stateTransition(x_prev,P_prev,u,x_hat,P_hat,F_x,F_u);
  cout<<"x_hat:"<<endl;
  cout<<x_hat<<endl;
  cout<<"F_x:"<<endl;
  cout<<F_x<<endl;
  cout<<"F_u:"<<endl;
  cout<<F_u<<endl;
  //2. test measurement prediction 
  x_hat << 0.822491645272271,0.407393371244097,2.32440805903468;
  vector<LineSegment> landmarks;
  vector<double> alpha = {-1.1761,-1.9039,0.7278,0.6966,0.7366};
  vector<double> r = {-1.1431,2.7635,-0.1290,0.2810,0.2757};
  for(int i=0;i<alpha.size();i++){
    landmarks.push_back(LineSegment(alpha[i],r[i]));
  }
  vector<LineSegment> z_hat;
  vector<Matrix23d> H;
  ekf.predictMeasurementsFromPriorState(landmarks,x_hat,z_hat,H);
  //print result 
  cout<<"z_hat"<<endl;
  for(auto& l:z_hat){
    cout<<l.alpha_<<' '<<l.r_<<endl;
  }
  cout<<"H"<<endl;
  for(auto& h:H){
    cout<<h<<endl;
  }
  //3. test measurement association 
  vector<LineSegment> z; 
  vector<Matrix2d> R;
  vector<pair<int,int>> matches;
  vector<double> alpha3{1.9017,-0.4802,2.3446,1.7224,-0.6169,1.4243,1.5699,-0.3538};
  vector<double> r3{-0.1816,-2.0154,-0.5815,0.7246,2.3699,-0.2202,2.5035,-1.0409};
  //landmarks
  landmarks.clear();
  for(int i=0; i<alpha3.size();i++){
    landmarks.push_back(LineSegment(alpha3[i],r3[i]));
  }
  //z
  alpha3=vector<double>{2.9897,-2.5337,0.2911,2.8105,0.4709};
  r3=vector<double>{1.7219,0.7474,0.5097,3.2831,3.0671};
  for(int i=0; i<alpha3.size();i++){
    z.push_back(LineSegment(alpha3[i],r3[i]));
  }
  //R,same size as z
  for(int i=0; i<alpha3.size(); i++){
    R.push_back(Matrix2d::Identity()/100);
  }
  //P_hat 
  P_hat = Matrix3d::Identity()/100;
  //x_hat
  x_hat << -3.01778205486161,-3.04928466715739,-1.08799857727348;
  ekf.predictMeasurementsFromPriorState(landmarks,x_hat,z_hat,H);
  ekf.associateObservationWithPrediction(z,R,z_hat,H,P_hat,matches);
  //output innovation 
  cout<<"v"<<endl;
  for(int i=0; i<matches.size(); i++){
    cout<< z[matches[i].first].vector()<<endl<<z_hat[matches[i].second].vector()<<endl;
  }
  cout<<"filtered H"<<endl;
  for(int i=0; i<matches.size(); i++){
    cout<<H[matches[i].second]<<endl;
  }
  //4. test one whole filter step
  //Posterior estimation 
  x_prev << 0.333542078036589,3.75724002564630,-0.671681351510224;
  P_prev << 0.1,0,0,0,0.1,0,0,0,0.1;
  Vector3d x;
  Matrix3d P; 
  //landmarks
  landmarks.clear();
  vector<double> alpha4{0.0063, 0.6537,-1.2716,-0.9752,-1.1639,-1.0172,2.3454,1.0894};
  vector<double> r4{0.3467,-2.5259,0.7271,1.6774,-2.8934,0.3542,1.1883,0.3709};
  landmarks.clear();
  for(int i=0; i<alpha4.size();i++){
    landmarks.push_back(LineSegment(alpha4[i],r4[i]));
  }
  //z
  z.clear();
  alpha4 = vector<double>{-2.5224,-1.7187,-0.5708,-0.4975,-0.6633,-0.1059,-0.2678,-1.4506};
  r4 = vector<double>{0.0380,5.2591,4.3229,4.7227,0.2601,3.4013,1.2589,2.9059};
  for(int i=0; i<alpha4.size();i++){
    z.push_back(LineSegment(alpha4[i],r4[i]));
  }
  //R
  R.clear();
  for(int i=0; i<alpha4.size(); i++){
    R.push_back(Matrix2d::Identity()/10);
  }
  u << 0.00189280253910503,0.00919748352913642;
  //correct
  //state transition
  ekf.stateTransition(x_prev,P_prev,u,x_hat,P_hat,F_x,F_u);
  cout<<"Prior estimation"<<endl;
  cout<<x_hat<<endl;
  cout<<P_hat<<endl;
  //correct
  //measurement prediction 
  ekf.predictMeasurementsFromPriorState(landmarks,x_hat,z_hat,H);
  cout<<"Measurement prediction"<<endl;
  for(auto& l:landmarks){
    cout<<l.alpha_<<" "<<l.r_<<endl;
  }
  for(auto& l:z_hat){
    cout<<l.alpha_<<' '<<l.r_<<endl;
  }
  cout<<"H"<<endl;
  for(auto& h:H){
    cout<<h<<endl;
  }
  //correct
  //measurement association 
  cout<<"Measurement association"<<endl;
  ekf.associateObservationWithPrediction(z,R,z_hat,H,P_hat,matches);
  cout<<"Size of matches: "<<matches.size()<<endl;
  cout<<"v"<<endl;
  for(int i=0; i<matches.size(); i++){
    cout<< z[matches[i].first].vector()-z_hat[matches[i].second].vector()<<endl;
  }
  ekf.compPosteriorEstimation(z, R, x_hat, P_hat, z_hat, H, matches, x, P);
  cout<<"Posterior state estimation"<<endl;
  cout<<x<<endl;
  cout<<"Posterior covariance estimation"<<endl;
  cout<<P<<endl;
#endif
}