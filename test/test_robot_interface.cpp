#include "ekf_slam/robot_interface.h"
#include "ekf_slam/laser_frame.h"
#include "ekf_slam/common.h"
using ekf_slam::RobotInterface;
using ekf_slam::LaserFrame;
using ekf_slam::LineSegment;
int main(int argc, char **argv){
  ros::init(argc, argv, "ekf slam");
  ros::NodeHandle nh("~");
  ros::Rate loopRate(10);
  
  RobotInterface::Ptr robotPtr = std::make_shared<ekf_slam::RobotInterface>(nh);
  
  while(ros::ok()){
    if(robotPtr->state_==RobotInterface::State::Initialized){
      cout<<"Initialized"<<endl;
    } else if(robotPtr->state_ == RobotInterface::State::Running){
      cout<<"Running"<<endl;
      shared_ptr<LaserFrame> laserPtr = robotPtr->getLaserFrame();
      Vector3d T_w_l_gt = robotPtr->robotPose();
    } else{
      cout<<"Lost"<<endl;
    }
    //robotPtr->setRobotVelocity(0,30);
    ros::spinOnce();
    loopRate.sleep();
  }
  return 0;
}