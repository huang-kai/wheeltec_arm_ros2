cmake_minimum_required(VERSION 3.8)
project(wheeltec_arm_pick)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(wheeltec_arm_pick_msgs REQUIRED)  

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include
)

set(wheeltec_six_arm_SRCS
  src/wheeltec_arm_six.cpp
)

add_executable(wheeltec_six_arm src/wheeltec_arm_six.cpp src/quaternion_solution.cpp)
#target_link_libraries(wheeltec_six_arm ${catkin_LIBRARIES})
#add_dependencies(wheeltec_six_arm ${PROJECT_NAME}_generate_messages_cpp)
#add_dependencies(wheeltec_six_arm wheeltec_tracker_pkg_generate_messages_cpp)
ament_target_dependencies(wheeltec_six_arm serial tf2_ros tf2 tf2_geometry_msgs sensor_msgs rclcpp nav_msgs std_msgs wheeltec_arm_pick_msgs "rclcpp") 

install(TARGETS
wheeltec_six_arm

DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
