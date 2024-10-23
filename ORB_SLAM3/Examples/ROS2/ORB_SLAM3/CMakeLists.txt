cmake_minimum_required(VERSION 3.8)
project(ORB_SLAM3)

set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages/")


include(CMakePrintHelpers)

set(ORB_SLAM3_DIR "../../../../ORB_SLAM3")  # Adjust this to your ORB_SLAM3 directory

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)

# Find OpenCV (ORB_SLAM3 depends on OpenCV)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)



include_directories(
  include()
  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus
  ${Pangolin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  )



set(LIBS 
  ${OpenCV_LIBS} 
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/libORB_SLAM3.so
  # -lboost_system
  )

add_executable(mono
src/mono.cpp 
src/mono_node.cpp
)

link_directories(
  ${ORB_SLAM3_DIR})






  # Link the ORB_SLAM3 library and dependencies
target_link_libraries(mono 
${LIBS}
)


# Specify ROS 2 package dependencies
ament_target_dependencies(mono
rclcpp
sensor_msgs
cv_bridge
nav_msgs
)

install(TARGETS
mono
DESTINATION lib/${PROJECT_NAME}
)


install(PROGRAMS
  src/camera_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)


# Install Python modules
install(
  DIRECTORY src/
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Install launch files (if applicable)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
