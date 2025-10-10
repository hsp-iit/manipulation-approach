cmake_minimum_required(VERSION 3.8)
project(segment_task_surfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(laser_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)
FIND_PACKAGE(YARP COMPONENTS os sig dev REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_eigen)

find_package(tf2_sensor_msgs REQUIRED)

find_package(rclcpp_action REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(rosidl_default_generators REQUIRED)


#include_directories(/opt/ros/humble/include/tf2_geometry_msgs)
#include_directories(/opt/ros/humble/include/tf2)
#include_directories(/opt/ros/humble/include/tf2_ros)
#include_directories(/opt/ros/humble/include/laser_geometry)
#include_directories(/opt/ros/humble/include/pcl_conversions)
#include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(/opt/ros/iron/include/tf2_geometry_msgs)
include_directories(/opt/ros/iron/include/tf2)
include_directories(/opt/ros/iron/include/tf2_ros)
include_directories(/opt/ros/iron/include/laser_geometry)
include_directories(/opt/ros/iron/include/pcl_conversions)
include_directories(include)
#add_subdirectory(interfaces)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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



add_executable(desk_detector src/segment_closest_desk.cpp)
ament_target_dependencies(desk_detector
  "rclcpp"
  "rclcpp_lifecycle"
  "tf2_ros"
  "tf2_msgs"
  "tf2"
  "tf2_sensor_msgs"
  "sensor_msgs"
  "visualization_msgs"
  "geometry_msgs"
  "pcl_ros"
  "tf2_eigen"
)
target_link_libraries(desk_detector ${PCL_LIBRARIES})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  param
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  desk_detector
  DESTINATION lib/${PROJECT_NAME})


ament_package()
