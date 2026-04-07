cmake_minimum_required(VERSION 3.8)
project(mpc_to)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(high_solver 
  src/high_solver.cpp #src/low_solver1.cpp
  src/kinematics_functions.cpp
  solver_to_high_disc/UR5_model/UR5_dyn_disc_phi_fun_jac_hess.c # discrete dynamics
  solver_to_high_disc/UR5_model/UR5_dyn_disc_phi_fun_jac.c # discrete dynamics
  solver_to_high_disc/UR5_model/UR5_dyn_disc_phi_fun.c # discrete dynamics
  # solver_to_high/UR5_model/UR5_expl_ode_fun.c # explicit dynamics
  # solver_to_high/UR5_model/UR5_expl_ode_hess.c # explicit dynamics
  # solver_to_high/UR5_model/UR5_expl_vde_forw.c # explicit dynamics
  # solver_to_high/UR5_model/UR5_expl_vde_adj.c # explicit dynamics
  # solver_to_high_disc/UR5_constraints/UR5_constr_h_fun_jac_uxt_zt_hess.c # no human
  solver_to_high_disc/UR5_constraints/UR5_constr_h_fun_jac_uxt_zt.c
  solver_to_high_disc/UR5_constraints/UR5_constr_h_fun.c
  # solver_to_high_disc/UR5_constraints/UR5_constr_h_0_fun_jac_uxt_zt_hess.c # no human
  solver_to_high_disc/UR5_constraints/UR5_constr_h_0_fun_jac_uxt_zt.c
  solver_to_high_disc/UR5_constraints/UR5_constr_h_0_fun.c
  solver_to_high_disc/UR5_cost/UR5_cost_ext_cost_0_fun.c
  solver_to_high_disc/UR5_cost/UR5_cost_ext_cost_0_fun_jac.c
  solver_to_high_disc/UR5_cost/UR5_cost_ext_cost_0_fun_jac_hess.c
  solver_to_high_disc/UR5_cost/UR5_cost_ext_cost_e_fun.c
  solver_to_high_disc/UR5_cost/UR5_cost_ext_cost_e_fun_jac.c
  solver_to_high_disc/UR5_cost/UR5_cost_ext_cost_e_fun_jac_hess.c
  solver_to_high_disc/UR5_cost/UR5_cost_ext_cost_fun.c
  solver_to_high_disc/UR5_cost/UR5_cost_ext_cost_fun_jac.c
  solver_to_high_disc/UR5_cost/UR5_cost_ext_cost_fun_jac_hess.c
  solver_to_high_disc/acados_solver_UR5.c
  # solver_to_high_disc/acados_sim_solver_UR5.c # explicit dynamics
  solver_to_high_disc/acados_solver_holder.cpp
)
target_include_directories(high_solver PUBLIC
  include
  include/acados
  include/acados_c
  include/blasfeo/include
	include/hpipm/include
	solver_to_high_disc/
	${rclcpp_INCLUDE_DIRS}
)
link_directories(~/Documents/acados/lib)

target_link_libraries(high_solver ${rclcpp_LIBRARIES}
  ~/Documents/acados/lib/libacados.so
  ~/Documents/acados/lib/libhpipm.so
  ~/Documents/acados/lib/libblasfeo.so
  #/solver/acados_solver_holder.cpp acados hpipm blasfeo
)
target_compile_features(high_solver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(high_solver rclcpp std_msgs sensor_msgs)
message(STATUS "Target my_node_cpp built successfully")
install(TARGETS high_solver DESTINATION lib/${PROJECT_NAME})

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
