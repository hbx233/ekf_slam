/**
 * Robot interface for the differential robot simulated in vrep with laser sensor 
 */
#ifndef ROBOT_INTERFACE_H_
#define ROBOT_INTERFACE_H_

#include "ekf_slam/common.h"
#include "ekf_slam/laser_frame.h"

#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose2D.h"
namespace ekf_slam{

class RobotInterface
{
  /**Robot State
   * Lost: RobotInterface find no robot to connect
   * Initialized: RobotInterface received all initialization message
   * Running: Robot is running and RobotInterface is receiving message from robot
   */
public:
  enum class State {Lost, Initialized, Running};
  //Initial state is lost, no connection
  State state_{State::Lost};
  bool running_flag = false;
  bool called_all_callback;
  
public:
  using Ptr = std::shared_ptr<RobotInterface>;

public:
  explicit RobotInterface(const ros::NodeHandle& nh);
  //Cannot copy
  RobotInterface(const RobotInterface& ) = delete;
  RobotInterface& operator=(const RobotInterface& ) = delete;
public:
  /** 
   * Getters of robot configuration
   */ 
  //Circular robot diameter 
  double robotDiameter() const;
  //Two wheels' diameter
  double robotwheelDiameter() const;
  //Distance between two wheels
  double robotwheelDistance() const;
  
  /**
   *  Getters of robot state(2D pose)
   */
  Vector3d checkAndReturnPose(const Vector3d& pose) const;
  void checkAndPublishPose(const Vector3d& pose, Vector3d& cache, const ros::Publisher& pub);
  //Transformation of Laser sensor frame in Robot reference frame
  Vector3d laserPoseInRobotFrame() const;
  //Robot's absolute pose in world frame
  Vector3d robotPose() const;
  //Robot's estimated pose in world frame
  Vector3d estimatedPose() const;
  void setEstimatedPose(const Vector3d& estimated_pose);
  //Robot's target pose in world frame
  Vector3d targetPose() const;
  void setTargetPose(const Vector3d& target_pose);
  
  //get and set visibility 
  bool estimatedPoseVisibility() const;
  void setEstimatedPoseVisibility(bool visible);
  bool targetPoseVisibility() const;
  void setTargetPoseVisibility(bool visible);
  /**
   * Getters of laser reading
   */
   shared_ptr<LaserFrame> getLaserFrame();
  
  /**
   * Robot command, given forward and angular velocity,
   * calculates right and left wheels' velocity and publish them
   */
  
  void setRobotVelocity(double v, double w);
  
private:
  //ros NodeHandle
  ros::NodeHandle nh_;
  
  //robot configuration 
  float robot_diameter_{0};
  
  float robot_wheel_diameter_{0};
  
  float robot_wheel_distance_{0};
  
  enum SubsAndPubs{laser_scan,T_r_l,T_w_r,T_w_e,T_w_t,estimated_vis,target_vis,l_vel,r_vel};
  //laser frame in robot reference frame 
  string topic_name_[9];
  int queue_size_[9]={10,10,10,10,10,10,10,10,10};
  
  LaserFrame::Ptr laser_frame_ptr_;
  ros::Subscriber sub_laser_scan;
  
  Vector3d T_robot_laser_;
  ros::Subscriber sub_T_robot_laser_;
  
  //robot frame in world reference frame, ground truth  
  Vector3d T_world_robot_;
  ros::Subscriber sub_T_world_robot_;
  
  //estimated robot frame in world reference frame 
  Vector3d T_world_estimated_;
  //ros::Subscriber sub_T_world_estimated_;
  ros::Publisher pub_T_world_estimated_;
  
  //target frame in world reference frame 
  Vector3d T_world_target_;
  //ros::Subscriber sub_T_world_target_;
  ros::Publisher pub_T_world_target_;
  
  //visibility publisher 
  bool estimated_pose_visibility_{false};
  ros::Publisher pub_estimated_pose_visibility_;
  
  bool target_pose_visibility_{false};
  ros::Publisher pub_target_pose_visibility_;
  
  double left_wheel_vel_{0};
  ros::Publisher pub_left_wheel_vel_;
  
  double right_wheel_vel_{0};
  ros::Publisher pub_right_wheel_vel_;

private:
  //Callback functions for laser scan, convert LaserScan message to 2D coordinate 
  void laser_scan_callback(const sensor_msgs::LaserScan& msg);
  void write_scan_to_file();
  
  //Callback functions for robot state
  void T_rl_callback(const geometry_msgs::Pose2D& msg);
  void T_wr_callback(const geometry_msgs::Pose2D& msg);
  //void T_we_callback(const geometry_msgs::Pose2D& msg);
  //void T_wt_callback(const geometry_msgs::Pose2D& msg);
  //convertion between Pose2D and SE2 
  inline Vector3d convertPose2DToVector(const geometry_msgs::Pose2D& msg);
  inline geometry_msgs::Pose2D convertVectorToPose2D(const Vector3d& s);
};
Vector3d RobotInterface::convertPose2DToVector(const geometry_msgs::Pose2D& msg)
{
  return Vector3d(msg.x,msg.y,msg.theta);
}

geometry_msgs::Pose2D RobotInterface::convertVectorToPose2D(const Vector3d& s)
{
  geometry_msgs::Pose2D pose2D;
  pose2D.x = s(0);
  pose2D.y = s(1);
  pose2D.theta = s(2);
}

}

#endif //ROBOT_INTERFACE_H_
