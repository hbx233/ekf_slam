add_library(ekf_slam SHARED
    robot_interface.cpp
    laser_frame.cpp
    line_detector.cpp
    line_segment.cpp
    ekf.cpp
)
target_link_libraries( ekf_slam
    ${THIRD_PARTY_LIBS}
)
