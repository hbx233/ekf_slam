#include "ekf_slam/laser_frame.h"
#include "ekf_slam/common.h"
#include "ekf_slam/line_detector.h"
#include "ekf_slam/line_segment.h"
using ekf_slam::LaserFrame;
using ekf_slam::LineSegment;
using ekf_slam::LineDetector;
int main(){
  //read data point 
  std::ifstream ifs;
  ifs.open("/home/hbx/data/laser_point.txt");
  if(!ifs){
    std::cerr<<"Unable to open the data file"<<endl;
    exit(1);
  }
  vector<Vector2d> laser_points;
  string line;
  while(std::getline(ifs,line)){
    std::istringstream iss(line);
    Vector2d laser_point;
    iss>>laser_point[0]>>laser_point[1];
    laser_points.push_back(laser_point);
  }
  //construct LaserFrame
  LaserFrame laser_frame(0,0,laser_points);
  LineDetector::Ptr line_detector_ptr=LineDetector::create(0.01,15);
  vector<LineSegment> line_segments;
  line_detector_ptr->detect(laser_frame.laser_points_,line_segments);
  line_detector_ptr->display_line_segments(laser_frame.laser_points_,line_segments);
  return 0;
}