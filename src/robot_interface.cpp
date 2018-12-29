#include "ekf_slam/common.h"
#include "ekf_slam/robot_interface.h"

namespace ekf_slam {

RobotInterface::RobotInterface(const ros::NodeHandle& nh) 
  : nh_(nh)
{
  // initialize the parameter server
  // robot configuration, get robot configuration from parameter server 
  nh_.getParam("robot/diameter", robot_diameter_);
  nh_.getParam("robot/wheel_diameter", robot_wheel_diameter_);
  nh_.getParam("robot/wheel_distance", robot_wheel_distance_);
  // laser scan reading 
  nh_.getParam("vrep/laser/name", topic_name_[laser_scan]);
  nh_.getParam("vrep/laser/queue", queue_size_[laser_scan]);
  // ground truth transformation from vrep  
  nh_.getParam("vrep/tf/laser/name", topic_name_[T_r_l]);
  nh_.getParam("vrep/tf/laser/queue", queue_size_[T_r_l]);
  nh_.getParam("vrep/tf/robot/name", topic_name_[T_w_r]);
  nh_.getParam("vrep/tf/robot/queue", queue_size_[T_w_r]);
  // estimated pose and target pose publisher parameter
  nh_.getParam("vrep/estimated_pose/name", topic_name_[T_w_e]);
  nh_.getParam("vrep/estimated_pose/queue", queue_size_[T_w_e]);
  nh_.getParam("vrep/target_pose/name", topic_name_[T_w_t]);
  nh_.getParam("vrep/target_pose/queue", queue_size_[T_w_t]);
  //visibility publisher
  nh_.getParam("vrep/visibility/estimated/name", topic_name_[estimated_vis]);
  nh_.getParam("vrep/visibility/estimated/queue", queue_size_[estimated_vis]);
  nh_.getParam("vrep/visibility/target/name", topic_name_[target_vis]);
  nh_.getParam("vrep/visibility/target/queue", queue_size_[target_vis]);
  // wheel velocity 
  nh_.getParam("cmd_vel/left/name",topic_name_[l_vel]);
  nh_.getParam("cmd_vel/left/queue",queue_size_[l_vel]);
  nh_.getParam("cmd_vel/right/name",topic_name_[r_vel]);
  nh_.getParam("cmd_vel/right/queue",queue_size_[r_vel]);
  
  //Connect subscriber 
  //laser scan
  sub_laser_scan = nh_.subscribe(topic_name_[laser_scan],queue_size_[laser_scan],&RobotInterface::laser_scan_callback,this);
  //ground truth transformation 
  sub_T_robot_laser_ = nh_.subscribe(topic_name_[T_r_l],queue_size_[T_r_l],&RobotInterface::T_rl_callback,this);
  sub_T_world_robot_ = nh_.subscribe(topic_name_[T_w_r],queue_size_[T_w_r],&RobotInterface::T_wr_callback,this);
  
  //Connect Publisher 
  //estimated and target pose 
  pub_T_world_estimated_ = nh_.advertise<geometry_msgs::Pose2D>(topic_name_[T_w_e],queue_size_[T_w_e]);
  pub_T_world_target_ = nh_.advertise<geometry_msgs::Pose2D>(topic_name_[T_w_t],queue_size_[T_w_t]);
  //estimated and target pose visibility 
  pub_estimated_pose_visibility_ = nh_.advertise<std_msgs::Bool>(topic_name_[estimated_vis],queue_size_[estimated_vis],true);
  pub_target_pose_visibility_ = nh_.advertise<std_msgs::Bool>(topic_name_[target_vis],queue_size_[target_vis],true);
  //velocity command 
  pub_left_wheel_vel_ = nh_.advertise<std_msgs::Float32>(topic_name_[l_vel],queue_size_[l_vel]);
  pub_right_wheel_vel_ = nh_.advertise<std_msgs::Float32>(topic_name_[r_vel],queue_size_[r_vel]);
  cout<<
  //Transfer to Initialized state
  state_ = State::Initialized;
}


double RobotInterface::robotDiameter() const
{
  if(state_ != State::Lost){
    return robot_diameter_;
  } else{
    ERROR_MSG("Robot Is Not Initialized");
    return 0;
  }
}

double RobotInterface::robotwheelDistance() const
{
  if(state_ != State::Lost){
    return robot_wheel_distance_;
  } else{
    ERROR_MSG("Robot Is Not Initialized");
    return 0;
  } 
}
double RobotInterface::robotwheelDiameter() const
{
  if(state_ != State::Lost){
    return robot_wheel_diameter_;
  } else{
    ERROR_MSG("Robot Is Not Initialized");
    return 0;
  }
}
//robot state
SE2 RobotInterface::checkAndReturnPose(const SE2& pose) const
{
  if(state_ == State::Running){
    return pose;
  } else{
    ERROR_MSG("Robot Is Not Running");
    return SE2();
  }
}
void RobotInterface::checkAndPublishPose(const SE2& pose, SE2& cache, const ros::Publisher& pub)
{
  if(state_ == State::Running){
    cache = pose;
    pub.publish(convertSE2ToPose2D(cache));
  } else{
    ERROR_MSG("Robot Is Not Running");
  }
}

SE2 RobotInterface::laserPoseInRobotFrame() const
{
  return checkAndReturnPose(T_robot_laser_);
}
SE2 RobotInterface::robotPose() const
{
  return checkAndReturnPose(T_world_robot_);
}

SE2 RobotInterface::estimatedPose() const
{
  return checkAndReturnPose(T_world_estimated_);
}
void RobotInterface::setEstimatedPose(const SE2& estimated_pose)
{
  checkAndPublishPose(estimated_pose, T_world_estimated_, pub_T_world_estimated_);
}

SE2 RobotInterface::targetPose() const
{
  return checkAndReturnPose(T_world_target_);
}
void RobotInterface::setTargetPose(const SE2& target_pose)
{
  checkAndPublishPose(target_pose, T_world_target_, pub_T_world_target_);
}

shared_ptr< LaserFrame > RobotInterface::getLaserFrame()
{
  if (state_ == State::Running && laser_frame_ptr_!=nullptr){
    return laser_frame_ptr_;
  } else{
    ERROR_MSG("Robot Is Not Running");
    return nullptr;
  }
}

bool RobotInterface::estimatedPoseVisibility() const
{
  if (state_ == State::Running){
    return estimated_pose_visibility_;
  } else{
    ERROR_MSG("Robot Is Not Running");
    return false;
  }
}

void RobotInterface::setEstimatedPoseVisibility(bool visible)
{
  if(state_ == State::Running){
    estimated_pose_visibility_ = visible;
    std_msgs::Bool vis;
    vis.data = estimated_pose_visibility_;
    pub_estimated_pose_visibility_.publish(vis);
  } else{
    ERROR_MSG("Robot Is Not Running");
  }
}

bool RobotInterface::targetPoseVisibility() const
{
  if (state_ == State::Running){
    return target_pose_visibility_;
  } else{
    ERROR_MSG("Robot Is Not Running");
    return false;
  }
}

void RobotInterface::setTargetPoseVisibility(bool visible)
{
  if(state_ == State::Running){
    target_pose_visibility_ = visible;
    std_msgs::Bool vis;
    vis.data = target_pose_visibility_;
    pub_target_pose_visibility_.publish(vis);
  } else{
    ERROR_MSG("Robot Is Not Running");
  }
}



void RobotInterface::setRobotVelocity(double v, double w)
{
  if(state_ == State::Running){
    //compute wheel velocity given the robot velocity 
    left_wheel_vel_ = (v+(robot_wheel_distance_/2)*w)/(robot_wheel_diameter_/2); 
    right_wheel_vel_= (v-(robot_wheel_distance_/2)*w)/(robot_wheel_diameter_/2);
    std_msgs::Float32 l_vel_msg;
    std_msgs::Float32 r_vel_msg;
    l_vel_msg.data = left_wheel_vel_;
    r_vel_msg.data = right_wheel_vel_;
    pub_left_wheel_vel_.publish(l_vel_msg);
    pub_right_wheel_vel_.publish(r_vel_msg);
  } else{
    ERROR_MSG("Robot Is Not Running");
  }
}
//Callback functions 
void RobotInterface::T_rl_callback(const geometry_msgs::Pose2D& msg)
{
  T_robot_laser_ = convertPose2DToSE2(msg);
}

// Robot State Callback functions 
void RobotInterface::T_wr_callback(const geometry_msgs::Pose2D& msg)
{
  T_world_robot_ = convertPose2DToSE2(msg);
}

// Laser Scan Callback function 
void RobotInterface::laser_scan_callback(const sensor_msgs::LaserScan& msg)
{
  //Convert Laser scan to point cloud in laser frame 
  static unsigned long id = 0;
  id++;
  laser_frame_ptr_ = std::make_shared<LaserFrame>(id,msg.header.stamp.sec);
  for(size_t i = 0; i < msg.ranges.size(); i++){
    auto angle = msg.angle_min + msg.angle_increment*i;
    Vector2d _point;
    _point[0] = msg.ranges[i] * std::cos(angle);
    _point[1] = msg.ranges[i] * std::sin(angle);
    laser_frame_ptr_->laser_points_.push_back(_point);
  }
  state_ = State::Running;
}
}


