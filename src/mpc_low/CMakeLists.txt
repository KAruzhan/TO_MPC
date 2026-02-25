cmake_minimum_required(VERSION 3.8)
project(mpc_low)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(low_solver 
  src/low_solver.cpp
  src/kinematics_functions.cpp
  solver_low/UR5_model/UR5_expl_ode_fun.c
  solver_low/UR5_model/UR5_expl_vde_forw.c
  solver_low/UR5_model/UR5_expl_vde_adj.c
  solver_low/UR5_constraints/UR5_constr_h_fun_jac_uxt_zt.c
  solver_low/UR5_constraints/UR5_constr_h_fun.c
  solver_low/acados_solver_UR5.c
  solver_low/acados_sim_solver_UR5.c
  solver_low/acados_solver_holder.cpp
)
target_include_directories(low_solver PUBLIC
  include
  include/acados
  include/acados_c
  include/blasfeo/include
  include/hpipm/include
  solver_low/
  ${rclcpp_INCLUDE_DIRS}
)
link_directories(~/Documents/acados/lib)

target_link_libraries(low_solver ${rclcpp_LIBRARIES}
  ~/Documents/acados/lib/libacados.so
  ~/Documents/acados/lib/libhpipm.so
  ~/Documents/acados/lib/libblasfeo.so
  #/solver/acados_solver_holder.cpp acados hpipm blasfeo
)
target_compile_features(low_solver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(low_solver rclcpp std_msgs sensor_msgs)
message(STATUS "Target my_node_cpp built successfully")
install(TARGETS low_solver DESTINATION lib/${PROJECT_NAME})

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
