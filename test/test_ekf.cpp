#include "ekf_slam/ekf.h"
#include "ekf_slam/common.h"
#include "ekf_slam/line_segment.h"
#include "ekf_slam/laser_frame.h"
#include <iomanip>
using ekf_slam::EKFilter;
using ekf_slam::LaserFrame;
using ekf_slam::LineSegment;

#define DEBUG(msg) cout<<"[DEBUG]: "<<msg<<endl;

int main(){
  EKFilter ekf(0.1,0.1,3.162277660168380);
  VectorXd x_prev(23);
  x_prev<<0.100724570041212,-0.181123038402756,0.143447730265768,1.97746724216702,0.394032704193871,2.54966593680547,2.42648195440154,-2.34045048467059,2.39780630831048,2.60057757621380,1.21832805951022,0.834898141731188,2.00559010992512,-2.52546774346283,0.359604784271156,-1.38847626615034,1.05929214426881,0.297825743225342,2.29422775067579,2.87786069691334,1.98540726160200,2.92424128441866,2.40362000368538;
  Vector2d u;
  u << -0.00307880517190656,0.0748320834617122;
  Vector3d T_prev(0.100724570041212,-0.181123038402756,0.14344773026576);
  MatrixXd P_prev(23,23);
  P_prev= MatrixXd::Zero(23,23);
  P_prev(0,0) = 0.0225; P_prev(1,1)=0.0225; P_prev(2,2)=0.04;
  for(int i=3;i<7;i++){
    P_prev(i,i) = 2 * std::pow(10,-8);
  }
  for(int i=2; i<10; i++){
    P_prev(3+2*i,3+2*i) = 0.04;
    P_prev(4+2*i,4+2*i) = 0.1225;
  }
  cout<<P_prev<<endl;
  //measurement prediction
  vector<double> alpha{1.97746724216702,2.54966593680547,-2.34045048467059,2.60057757621380,0.834898141731188,-2.52546774346283,-1.38847626615034,0.297825743225342,2.87786069691334,2.92424128441866};
  vector<double> r{0.394032704193871,2.42648195440154,2.39780630831048,1.21832805951022,2.00559010992512,0.359604784271156,1.05929214426881,2.29422775067579,1.98540726160200,2.40362000368538};
  vector<LineSegment> map_prev;
  for(int i=0;i<alpha.size();i++){
    map_prev.push_back(LineSegment(alpha[i],r[i]));
  }
  //z
  alpha = vector<double>{1.30916615390182,1.89793368946211,-3.02665603476663,1.94100580600647,0.176508057382488,3.09664757613740,-2.04276292857275,-0.360530839748197,2.21160494898571,2.27172531241573};
  r = vector<double>{0.395029034489874,2.43490455958245,2.41052975901097,1.24043568513749,1.96786097725679,0.384495330121063,1.04618905918672,2.24490174213010,2.01579993422366,2.41821379304755};
  vector<LineSegment> z(alpha.size());
  for(int i=0;i<alpha.size();i++){
    z[i] = LineSegment(alpha[i],r[i]);
  }
  //R
  vector<Matrix2d> R(z.size());
  for(int i=0;i<z.size();i++){
    Matrix2d temp = Matrix2d::Zero();
    temp(0,0) = 0.000004;
    temp(1,1) = 0.0001;
    R[i] = temp;
  }
  Vector3d T_posterior;
  vector<LineSegment> map_posterior;
  MatrixXd P_posterior;
  ekf.ekfOneStep(T_prev,u,z,R,map_prev,P_prev,T_posterior,map_posterior,P_posterior);
  cout<<"x_prior"<<endl;
  cout<<ekf.x_prior<<endl;
  cout<<"P_prior"<<endl;
  cout<<ekf.P_prior<<endl;
  for(auto& l:ekf.z_predict){
    cout<<l[0]<<' '<<l[1]<<endl;
  }
  for(auto& h:ekf.H){
    cout<<h<<endl;
  }
  for(auto& m:ekf.matches){
    cout<<m.first<<' '<<m.second<<endl;
  }
  cout<<ekf.x_posterior<<endl;
  cout<<P_posterior<<endl;
}