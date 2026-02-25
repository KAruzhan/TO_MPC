// standard
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_UR5.h"

// blasfeo
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

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
  // global data
  UR5_solver_capsule * acados_ocp_capsule;
public:
  my_NMPC_solver(int n, int number_of_current_steps_specified) {
    number_of_current_steps = number_of_current_steps_specified;
    num_steps = n; // set number of real-time iterations
    UR5_solver_capsule * my_acados_ocp_capsule = UR5_acados_create_capsule();
    acados_ocp_capsule = my_acados_ocp_capsule;
    // there is an opportunity to change the number of shooting intervals in C without new code generation
    //int N = UR5_N;
    int N = number_of_current_steps; // set variable step
    // allocate the array and fill it accordingly
    double stemps_array[number_of_current_steps]={0.0}; // create array of maximum length
    for (int i=0;i<number_of_current_steps;i++) stemps_array[i] = 1.0;
    
    printf("Trying to create solver\n");
    //double* new_time_steps = NULL; need to set array if N steps is changed
    double* new_time_steps = stemps_array;
    int status = UR5_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);

    if (status)
    {
        printf("UR5_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }
    printf("Solver created\n");
    
  }

  int solve_my_mpc(double current_joint_position[6], double current_human_position[56], double current_joint_goal[6], double cgoal[3], double results[31], double trajectory_first[66], double full_trajectory[70]) {
    int status = -1;
    int N = number_of_current_steps;

    ocp_nlp_config *nlp_config = UR5_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims *nlp_dims = UR5_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in *nlp_in = UR5_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out *nlp_out = UR5_acados_get_nlp_out(acados_ocp_capsule);
    ocp_nlp_solver *nlp_solver = UR5_acados_get_nlp_solver(acados_ocp_capsule);
    void *nlp_opts = UR5_acados_get_nlp_opts(acados_ocp_capsule);

    // initial condition
    int idxbx0[NBX0];  
    for (int i=0;i<6;i++) idxbx0[i] = i; // Fix only thetas from 1 to 6
    // Set constraints for initial state
    double lbx0[NBX0] = {0.0};
    double ubx0[NBX0] = {0.0};
    for (int i=0;i<6;i++)  {
        lbx0[i] = current_joint_position[i];
        ubx0[i] = current_joint_position[i];
    }

    // set constraints for final states
    double x_goal[NBX0] = {0.0}; // set all to 0.0, then change the position
    for (int i=0;i<6;i++)  {
        x_goal[i] = current_joint_goal[i];
    }

    // Slacks - no slack variables for pure time optimal cost
    /*int my_NSH = NH-3;
    int idxsh[my_NSH];
    for (int i=0;i<my_NSH;i++) idxsh[i]=i+3; // skiped first 3 constrains and set all other as slack
    double lh[NH];
    double uh[NH];
    for (int i=0;i<NH;i++) lh[i]=-10e6;
    for (int i=0;i<NH;i++) uh[i]=0.0;
    printf("*********\n\n %i %i %i\n*********\n\n", NSH, my_NSH, NH);*/


    int idxbx_step[NBX];
    for (int i=0;i<7;i++) idxbx_step[i] = i; // Fix only thetas from 1 to 6 and dt
    double pi=3.1415926;
    //double lbx_step[7] = {-2*pi, -2*pi, -2*pi, -2*pi, -2*pi, -2*pi, 0.01};
    //double ubx_step[7] = {2*pi, 2*pi, 2*pi, 2*pi, 2*pi, 2*pi, 1.000};
    
    double lbx_step[7] = {-2*pi, -2*pi, -2*pi, -3.4034, -pi/2, -2*pi, 0.005};
    double ubx_step[7] = {2*pi, 2*pi, 2*pi, 0.2618, pi/2, 2*pi, 0.500};


    int idxbu[NBU];
    for (int i=0;i<NBU;i++) idxbu[i] = i; // Fix only omega from 1 to 6 and ddt and slack
    double lbu[13] = {-1.2, -1.2, -1.2, -1.2, -1.2, -1.2, 0.0, -10e8, -10e8, -10e8, -10e8, -10e8, -10e8};
    double ubu[13] = {1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    for (int ii = 0; ii < N; ii++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "idxbu", idxbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "ubu", ubu);
        //ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "lh", lh);
        //ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "uh", uh);
        //ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "idxsh", idxsh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "idxbx", idxbx_step);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "lbx", lbx_step);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "ubx", ubx_step);
    }

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "lbx", x_goal);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "ubx", x_goal);
    // initialization for state values
    double x_init[NX]={0.0}; // set all to 0.0, then change the position
    for (int i=0;i<6;i++)  {
        x_init[i] = current_joint_position[i];
    }
    x_init[6] = 0.5; // initial dt

    // initial value for control input
    double u0[NU] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 6 vels 1 dt 6 slack

    // set parameters
    double p[NP] = {0.0};

    // set parameters
    for (int i = 0; i < 56; i++) p[i] = current_human_position[i];
    p[56] = cgoal[0]; p[57] = cgoal[1]; p[58] = cgoal[2];

    for (int ii = 0; ii <= N; ii++)
    {
        UR5_acados_update_params(acados_ocp_capsule, ii, p, NP);
    }

    // prepare evaluation
    int NTIMINGS = num_steps; // set number of real-time iterations
    double exec_time = 0.0;
    double kkt_norm_inf;
    double elapsed_time;
    int sqp_iter;

    double xtraj[NX * (N+1)];
    double utraj[NU * N];


    // solve ocp in loop
    int rti_phase = 0;

    for (int ii = 0; ii < NTIMINGS; ii++)
    {
        // initialize solution
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init); 
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
        }
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x_init);
        ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &rti_phase);
        status = UR5_acados_solve(acados_ocp_capsule);
        ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
        exec_time = exec_time + elapsed_time;
    }

    /* print solution and statistics */
    for (int ii = 0; ii <= nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii*NX]);
    for (int ii = 0; ii < nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii*NU]);
    printf("\nsolved ocp %d times, solution printed above\n\n", NTIMINGS);
    
    //***************************************
    /*results[30] = 0.0; // NaN marker
    for (int i=0;i<6;i++) {
        if (xtraj[6+i]!=xtraj[6+i]) results[30] = 1.0;
        else {
            results[i] = xtraj[6+i];
        }
    }
    for (int i=0;i<6;i++) {
        if (xtraj[NX+6+i]!=xtraj[NX+6+i]) results[30] = 1.0;
        else {
            results[i+6] = xtraj[NX+6+i];
        }
    }
    for (int i=0;i<6;i++) {
        if (xtraj[2*NX+6+i]!=xtraj[2*NX+6+i]) results[30] = 1.0;
        else {
            results[i+13] = xtraj[2*NX+6+i];
        }
    }
    printf("timings %f %f %f %f %f %f %f %f %f %f \n", utraj[6], utraj[NU+6], utraj[2*NU+6], utraj[3*NU+6], utraj[4*NU+6],utraj[5*NU+6], utraj[6*NU+6], utraj[7*NU+6], utraj[8*NU+6],utraj[9*NU+6]);
    results[12] = utraj[6];results[20] = utraj[NU+6];results[26] = utraj[2*NU+6];
    results[27] = sqp_iter; results[28] = exec_time*1000; results[29] = kkt_norm_inf;*/
    
    //***************************************
    for (int j=0;j<N;j++) {
        for (int i=0;i<6;i++) full_trajectory[7*j+i] = xtraj[NX*j+i];
        //full_trajectory[7*j+6] = utraj[NU*j+6];
        full_trajectory[7*j+6] = xtraj[NX*j+6]; // dt stored in states
    }

    //***************************************
    for (int j=0;j<=N;j++) for (int i=0;i<6;i++) trajectory_first[6*j+i] = xtraj[NX*j+i];

    if (status == ACADOS_SUCCESS)
    {
        printf("UR5_acados_solve(): SUCCESS!\n");
    }
    else
    {
        printf("UR5_acados_solve() failed with status %d.\n", status);
        printf("params %d %d %d %d %d %d  %d \n", N, NBX, NBU, NH, NP, NU, NBX0);
    }

    // get solution
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);

    //printf("\nSolver info:\n");
    printf(" SQP iterations %2d  minimum time for %d solve %f [ms]  KKT %e \n",
           sqp_iter, NTIMINGS, exec_time*1000, kkt_norm_inf);
    results[0] = sqp_iter;
    results[1] = NTIMINGS;
    results[2] = exec_time*1000;
    results[3] = kkt_norm_inf;
    results[4] = xtraj[6]; // [q1, q2, q3, q4, q5, q6, dt]
    return status;
  }  // end of 'solve' function

  int reset_solver(){
    int status = -1;
    // free solver
    status = UR5_acados_free(acados_ocp_capsule);
    if (status) {
        printf("UR5_acados_free() returned status %d. \n", status);
    }
    // free solver capsule
    status = UR5_acados_free_capsule(acados_ocp_capsule);
    if (status) {
        printf("UR5_acados_free_capsule() returned status %d. \n", status);
    }
    return status;
  }


};
