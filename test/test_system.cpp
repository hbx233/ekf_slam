#include "ekf_slam/system.h"

using ekf_slam::EKFilter;
using ekf_slam::LaserFrame;
using ekf_slam::LineDetector;
using ekf_slam::LineSegment;
using ekf_slam::Map;
using ekf_slam::System;

int main(){
  //create ekfilter
  EKFilter::Ptr ekfilter = std::make_shared<ekfilter>(0.1,0.25,);
}