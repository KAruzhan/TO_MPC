#ifndef ACADOS_SOLVER_HOLDER_HPP_
#define ACADOS_SOLVER_HOLDER_HPP_

// Standard
#include <stdio.h>
#include <stdlib.h>
// Acados
#include <acados/utils/print.h>
#include <acados/utils/math.h>
#include <acados_c/ocp_nlp_interface.h>
#include <acados_c/external_function_interface.h>
#include <acados_solver_UR5.h>
// Blasfeo
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

#include <math.h>
#include <eigen3/Eigen/Dense>

Eigen::MatrixXf get_cpose(float q1, float q2, float q3, float q4, float q5, float q6);

Eigen::MatrixXf get_velocity(float q1, float q2, float q3, float q4, float q5, float q6,
                             float u_1, float u_2, float u_3, float u_4, float u_5, float u_6);

#define NX     UR5_NX
#define NZ     UR5_NZ
#define NU     UR5_NU
#define NP     UR5_NP
#define NBX    UR5_NBX
#define NBX0   UR5_NBX0
#define NBU    UR5_NBU
#define NSBX   UR5_NSBX
#define NSBU   UR5_NSBU
#define NSH    UR5_NSH
#define NSG    UR5_NSG
#define NSPHI  UR5_NSPHI
#define NSHN   UR5_NSHN
#define NSGN   UR5_NSGN
#define NSPHIN UR5_NSPHIN
#define NSBXN  UR5_NSBXN
#define NS     UR5_NS
#define NSN    UR5_NSN
#define NG     UR5_NG
#define NBXN   UR5_NBXN
#define NGN    UR5_NGN
#define NY0    UR5_NY0
#define NY     UR5_NY
#define NYN    UR5_NYN
#define NH     UR5_NH
#define NPHI   UR5_NPHI
#define NHN    UR5_NHN
#define NPHIN  UR5_NPHIN
#define NR     UR5_NR

class my_NMPC_solver {
private:
    int num_steps;
    int number_of_current_steps;
    UR5_solver_capsule *acados_ocp_capsule;

public:
    my_NMPC_solver(int n, int number_of_current_steps_specified);
    int solve_my_mpc(double current_joint_position[6],
        // double current_joint_velocity[6],
        // double current_human_position[56], 
        double current_joint_goal[6], double tracking_goal[60], double cgoal[3], double results[22], double my_weights[10], double full_trajectory[140]);
    int reset_solver();
};

#endif // ACADOS_SOLVER_HOLDER_HPP_
