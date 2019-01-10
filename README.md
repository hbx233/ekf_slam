# Line-Based EKF SLAM System for Indoor Environment

## 1. Introduction:

This project implemented an EKF SLAM system for indoor differential robot, which use laser range finder as its main sensor. Assume no prior knowledge of environment, can simultaneously build a map of lines and localize robot, no limitation of map size. 

## 2. V-REP Simulation Environment: 

This project V-REP 3.5.0 as the simulation environment, the scene file is provided in scene/scene.ttt 

## 3. Prerequisites 

Built and tested on Ubuntu 16.04

Dependencies: 

1. ROS Kinetic 
2. Eigen: 3.3.5

## 4. Run Example:

1. Launch roscore

```bash
roscore
```

2. Open V-REP software and open the provided scene file /scene/scene.ttt
3. launch EKF SLAM system 

```shell
roslaunch ekf_slam ekf_slam.launch
```

## Mathematical Derivation 
The complete mathematical derivation of whole system can be found in _Mathematical Derivation.pdf_ file

## Result:

The white line is ground truth of robot trajectory, and the yellow line is posterior estimation of robot trajectory. 

#### Figure 1 Normal trajectory  

![](./figure/normal.jpeg)

#### Figure 2 Trajectory with unexpected collision

![](./figure/collision.jpeg)

The trajectory contains four unexpected collision. When collision occurs, the posterior estimation will not be as accurate as normal, but the system can recover from collision error afterwards, which shows the robustness of the system





