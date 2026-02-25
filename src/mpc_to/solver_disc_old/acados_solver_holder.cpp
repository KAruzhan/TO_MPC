#include "acados_solver_holder.hpp"

my_NMPC_solver::my_NMPC_solver(int n, int number_of_current_steps_specified) {
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
    
    //double* new_time_steps = NULL; need to set array if N steps is changed
    double* new_time_steps = stemps_array;
    int status = UR5_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);

    if (status)
    {
        printf("UR5_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }
  }

int my_NMPC_solver::solve_my_mpc(double current_joint_position[6], 
    // double current_human_position[56], 
    double current_joint_goal[6], double tracking_goal[60], double cgoal[3], double results[16], double my_weights[10], double full_trajectory[140]) {
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
    for (int i=0;i<12;i++) idxbx_step[i] = i; // Fix only thetas from 1 to 6 and dt
    double pi=3.1415926;
    //double lbx_step[7] = {-2*pi, -2*pi, -2*pi, -2*pi, -2*pi, -2*pi, 0.01};
    //double ubx_step[7] = {2*pi, 2*pi, 2*pi, 2*pi, 2*pi, 2*pi, 1.000};
    
    //double lbx_step[7] = {-2*pi, -2*pi, -2*pi, -3.4034, -pi/2, -2*pi, 0.005};
    //double ubx_step[7] = {2*pi, 2*pi, 2*pi, 0.2618, pi/2, 2*pi, 0.500};
    
    double lbx_step[12] = {-2*pi, -2*pi, -2*pi, -2*pi, -2*pi, -2*pi, -1.2, -1.2, -1.2, -1.2, -1.2, -1.2};
    double ubx_step[12] = {2*pi, 2*pi, 2*pi, 2*pi, 2*pi, 2*pi, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2};


    int idxbu[NBU];
    for (int i=0;i<NBU;i++) idxbu[i] = i; // Fix only omega from 1 to 6 and ddt and slack
    double lbu[NBU] = {-4.4,-4.4,-4.4,-4.4,-4.4,-4.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double ubu[NBU] = { 4.4, 4.4, 4.4, 4.4, 4.4, 4.4, 10e8, 10e8, 10e8, 10e8, 10e8, 10e8};
    

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

    // initial value for control input
    double u0[NU] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 6 vels 1 dt 6 slack

    // Set parameters
    // double p[NP] = {0.0};
    // for (int i = 0; i < 56; i++) p[i] = current_human_position[i];
    // p[56] = cgoal[0]; p[57] = cgoal[1]; p[58] = cgoal[2];
    // for (int ii = 0; ii <= N; ii++)
    // {
    //     UR5_acados_update_params(acados_ocp_capsule, ii, p, NP);
    // }

    double gamma = 1.6;

    for (int ii = 0; ii < N; ii++)
    {
        double w = pow(gamma, ii);
        UR5_acados_update_params(acados_ocp_capsule, ii, &w, 1);
    }


    // prepare evaluation
    int NTIMINGS = num_steps;
    double exec_time = 0.0;
    double kkt_norm_inf;
    double elapsed_time;
    int sqp_iter;

    double xtraj[NX * (N+1)];
    double utraj[NU * N];

    // solve ocp in loop
    int rti_phase = 0;
    double ocp_cost = 0.0;
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
                
        ocp_nlp_eval_cost(nlp_solver,nlp_in,nlp_out); 
        ocp_nlp_get(nlp_config, nlp_solver,"cost_value", &ocp_cost);
    }
    

    /* print solution and statistics */
    for (int ii = 0; ii <= nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii*NX]);
    for (int ii = 0; ii < nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii*NU]);
    
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

    /*if (status == ACADOS_SUCCESS)
    {
        printf("UR5_acados_solve(): SUCCESS!\n");
    }
    else
    {
        printf("UR5_acados_solve() failed with status %d.\n", status);
        printf("params %d %d %d %d %d %d  %d \n", N, NBX, NBU, NH, NP, NU, NBX0);
    }*/

    // get solution
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);

    //***************************************
    results[15] = 0.0; // NaN marker
    // no joint position data in results
    /*for (int i=0;i<6;i++) {
        if (xtraj[i+6]!=xtraj[i+6]) results[15] = 1.0; // x = [theta(6) omega(6)]
        else results[i] = xtraj[i+6];
        if (xtraj[i+6+12]!=xtraj[i+6+12]) results[15] = 1.0; // second horizon
        else results[i+6] = xtraj[i+6+12];
    }*/

    results[12] = sqp_iter; results[13] = exec_time*1000; results[14] = kkt_norm_inf;

    for (int j=0;j<N;j++) { // here we fix number of steps to pass to low-level controller to 10 instead of 20
        for (int i=0;i<6;i++) full_trajectory[7*j+i] = xtraj[NX*j+i]; // q[6]
        full_trajectory[7*j+6] = 1.0; // implicit unit time step
    }

    //***************************************
    //for (int j=0;j<=N;j++) for (int i=0;i<6;i++) trajectory_first[6*j+i] = xtraj[NX*j+i];

    //printf("\nSolver info:\n");
    /*printf(" SQP iterations %2d  minimum time for %d solve %f [ms]  cost %f \n",
           sqp_iter, NTIMINGS, exec_time*1000, ocp_cost);*/

    results[0] = 1.0;
    results[1] = NTIMINGS;
    results[2] = ocp_cost;
    return status;
  }  // end of 'solve' function

int my_NMPC_solver::reset_solver(){
    int status = -1;
    // free solver
    status = UR5_acados_free(acados_ocp_capsule);
    /*if (status) {
        printf("UR5_acados_free() returned status %d. \n", status);
    }*/
    // free solver capsule
    status = UR5_acados_free_capsule(acados_ocp_capsule);
    /*if (status) {
        printf("UR5_acados_free_capsule() returned status %d. \n", status);
    }*/
    return status;
}
