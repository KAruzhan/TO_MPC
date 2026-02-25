cmake_minimum_required(VERSION 3.8)
project(ur_ros2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
#find_package(actionlib REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
#find_package(ur_msgs REQUIRED)
find_package(tf2 REQUIRED)
  #roscpp
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)
link_libraries(pthread)

add_executable(ur_driver 
  #src/ur_ros_wrapper.cpp
    #src/ur_driver.cpp
    src/ur_velocity_controller.cpp
    src/ur_realtime_communication.cpp
    src/ur_communication.cpp
    src/robot_state.cpp
    src/robot_state_RT.cpp
    src/do_output.cpp 
)

ament_target_dependencies(ur_driver hardware_interface controller_manager control_msgs geometry_msgs sensor_msgs trajectory_msgs)# ur_msgs)
install(TARGETS ur_driver
  DESTINATION lib/${PROJECT_NAME})

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
