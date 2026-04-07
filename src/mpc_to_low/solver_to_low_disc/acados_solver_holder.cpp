#include "acados_solver_holder.hpp"

my_NMPC_solver::my_NMPC_solver(int n, int number_of_current_steps_specified) {
    number_of_current_steps = number_of_current_steps_specified;
    num_steps = n; // set number of real-time iterations
    has_prev_solution = false;
    UR5_solver_capsule * my_acados_ocp_capsule = UR5_acados_create_capsule();
    acados_ocp_capsule = my_acados_ocp_capsule;
    // there is an opportunity to change the number of shooting intervals in C without new code generation
    //int N = UR5_N;
    int N = number_of_current_steps; // set variable step
    // allocate the array and fill it accordingly
    double* stemps_array = (double*) malloc(number_of_current_steps * sizeof(double)); // create array of maximum length
    for (int i=0; i<number_of_current_steps; i++) 
        stemps_array[i] = 0.05;
    
    //double* new_time_steps = NULL; need to set array if N steps is changed
    int status = UR5_acados_create_with_discretization(acados_ocp_capsule, N, stemps_array);
    free(stemps_array);

    if (status)
    {
        printf("UR5_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    prev_xtraj = new double[(N + 1) * NX]();
    prev_utraj = new double[N * NU]();
  }

int my_NMPC_solver::solve_my_mpc(double current_joint_position[6],
    double current_joint_velocity[6],
    double current_human_position[56],
    double current_joint_goal[6], double tracking_goal[60], double cgoal[3], double results[22], double my_weights[10]) {
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
    for (int i = 0; i < NBX0; i++) idxbx0[i] = i;
    // Set constraints for initial state (positions and velocities, NBX0=12)
    double lbx0[NBX0] = {0.0};
    double ubx0[NBX0] = {0.0};
    for (int i = 0; i < 6; i++) {
        lbx0[i] = current_joint_position[i];
        ubx0[i] = current_joint_position[i];
        lbx0[6 + i] = current_joint_velocity[i];
        ubx0[6 + i] = current_joint_velocity[i];
    }


    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);
    
    
    // initialization for state values
    double x_init[NX]={0.0}; // set all to 0.0, then change the position
    for (int i=0;i<6;i++)  {
        x_init[i] = current_joint_position[i];
        x_init[6+i] = current_joint_velocity[i];
    }

    // initial value for control input
    double u0[NU] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 6 accel + 6 pos slacks + 6 vel slacks

    // Override the code-generated terminal hard constraint so it follows the live horizon.
    int idxbxN[NBXN];
    for (int i = 0; i < NBXN; i++) idxbxN[i] = i;
    double lbxN[NBXN] = {0.0};
    double ubxN[NBXN] = {0.0};
    for (int i = 0; i < 6; i++) {
        const double q_terminal = tracking_goal[(N - 1) * 6 + i];
        lbxN[i] = q_terminal;
        ubxN[i] = q_terminal;
        lbxN[6 + i] = 0.0;
        ubxN[6 + i] = 0.0;
    }
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "idxbx", idxbxN);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "lbx", lbxN);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "ubx", ubxN);

    // Set parameters: p[0]=gamma^ii, p[1..6]=joint_goal, p[7..62]=human_spheres, p[63..65]=cgoal
    double gamma = 3.0;

    for (int ii = 0; ii < N; ii++)
    {
        double p_stage[NP];
        for (int j=0;j<NP;j++) p_stage[j]=0.0;
        p_stage[0] = pow(gamma, ii);
        for (int j = 0; j < 6; j++) p_stage[j + 1] = tracking_goal[ii * 6 + j];
        for (int j = 0; j < 56; j++) p_stage[j + 7] = current_human_position[j];
        p_stage[63] = cgoal[0]; p_stage[64] = cgoal[1]; p_stage[65] = cgoal[2];
        UR5_acados_update_params(acados_ocp_capsule, ii, p_stage, NP);
    }


    // prepare evaluation
    int NTIMINGS = 1;
    double exec_time = 0.0;
    double kkt_norm_inf;
    double elapsed_time;
    int sqp_iter;

    double xtraj[NX * (N+1)];
    double utraj[NU * N];

    // Warm-start or cold-start initialization
    if (has_prev_solution) {
        // Shift previous solution by one step: stage k gets prev stage k+1
        for (int i = 0; i < N - 1; i++) {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", &prev_xtraj[(i + 1) * NX]);
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", &prev_utraj[(i + 1) * NU]);
        }
        // Last interval: replicate terminal state/control
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N - 1, "x", &prev_xtraj[N * NX]);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N - 1, "u", &prev_utraj[(N - 1) * NU]);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N,     "x", &prev_xtraj[N * NX]);
    } else {
        // Cold start on first call
        for (int i = 0; i < N; i++) {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init);
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
        }
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x_init);
    }

    // solve ocp in loop
    int rti_phase = 0;
    double ocp_cost = 0.0;
    for (int ii = 0; ii < NTIMINGS; ii++)
    {
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

    // Store solution for warm-starting next call
    for (int ii = 0; ii <= N; ii++)
        for (int j = 0; j < NX; j++) prev_xtraj[ii*NX + j] = xtraj[ii*NX + j];
    for (int ii = 0; ii < N; ii++)
        for (int j = 0; j < NU; j++) prev_utraj[ii*NU + j] = utraj[ii*NU + j];
    has_prev_solution = true;
    
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
    for (int i=0;i<6;i++) {
        if (xtraj[i+6]!=xtraj[i+6]) results[15] = 0.0; // x = [theta(6) omega(6)]
        else results[i] = xtraj[i+6];
        if (xtraj[i+6+12]!=xtraj[i+6+12]) results[15] = 0.0; // second horizon (x_1)
        else results[i+6] = xtraj[i+6+NX];
    }

    results[12] = sqp_iter; results[13] = exec_time*1000; results[14] = kkt_norm_inf;
    //***************************************
    //for (int j=0;j<=N;j++) for (int i=0;i<6;i++) trajectory_first[6*j+i] = xtraj[NX*j+i];

    //printf("\nSolver info:\n");
    printf(" SQP iterations %2d  minimum time for %d solve %f [ms]  cost %f \n",
           sqp_iter, NTIMINGS, exec_time*1000, ocp_cost);

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
