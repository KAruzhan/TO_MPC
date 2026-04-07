#include "acados_solver_holder.hpp"
#include <vector>

my_NMPC_solver::my_NMPC_solver(int n, int number_of_current_steps_specified) {
    number_of_current_steps = number_of_current_steps_specified;
    num_steps = n; // set number of real-time iterations
    UR5_solver_capsule * my_acados_ocp_capsule = UR5_acados_create_capsule();
    acados_ocp_capsule = my_acados_ocp_capsule;
    // there is an opportunity to change the number of shooting intervals in C without new code generation
    int N = number_of_current_steps;
    // allocate the array and fill it accordingly
    double* new_time_steps = (double*) malloc(number_of_current_steps * sizeof(double));
    for (int i = 0; i < number_of_current_steps; i++) new_time_steps[i] = 0.05;
    int status = UR5_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);
    free(new_time_steps);

    if (status)
    {
        printf("UR5_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    has_prev_solution = false;
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
    // Set constraints for initial state
    double lbx0[NBX0] = {0.0};
    double ubx0[NBX0] = {0.0};
    for (int i = 0; i < 6; i++) {
        lbx0[i] = current_joint_position[i];
        ubx0[i] = current_joint_position[i];
    }
    for (int i = 0; i < 6; i++) {
        lbx0[6 + i] = current_joint_velocity[i];
        ubx0[6 + i] = current_joint_velocity[i];
    }

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

    // initialization for state values
    double x_init[NX] = {0.0}; // set all zeros
    for (int j=0;j<6;j++) {
        x_init[j] = current_joint_position[j];
        x_init[6+j] = current_joint_velocity[j];
    }

    // initial value for control input
    double u0[NU]={0.0}; // set all zeros
    
    double W[(NU+NX)*(NU+NX)]= {0.0};
    double WN[(NX)*(NX)] = {0.0};
    
    for (int ii = 0; ii < (NX); ii++) W[ii+ii*(NU+NX)] = 100.0;
    for (int ii = NU; ii < (NU+NX); ii++) W[ii+ii*(NU+NX)] = 1.0;
    
    for (int ii=0;ii<N;ii++) {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "W", W);
    }
    //ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", WN);
    
    double y_ref[NY] = {0.0};
    for (int i=0;i<6;i++) y_ref[i] = current_joint_goal[i];
    double y_ref_N[NYN] = {0.0};
    for (int i=0;i<6;i++) y_ref_N[i] = current_joint_goal[i];
    
    // Set goal
    for (int ii=0;ii<N;ii++) {
        for (int i=0;i<6;i++) y_ref[i] = tracking_goal[ii*6+i]; // Check if OK???
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "yref", y_ref);
    }
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", y_ref_N);
    
    // Set parameters
    double p[NP] = {0.0};
    for (int i = 0; i < 56; i++) p[i] = current_human_position[i];
    p[56] = cgoal[0]; p[57] = cgoal[1]; p[58] = cgoal[2];
    for (int ii = 0; ii <= N; ii++)
    {
        UR5_acados_update_params(acados_ocp_capsule, ii, p, NP);
    }

    (void)my_weights;

    // prepare evaluation
    int NTIMINGS = 1;
    double exec_time = 0.0;
    double kkt_norm_inf;
    double elapsed_time;
    int sqp_iter;

    std::vector<double> xtraj(NX * (N + 1), 0.0);
    std::vector<double> utraj(NU * N, 0.0);

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

    /*printf("\n--- xtraj ---\n");
    d_print_exp_tran_mat( NX, N+1, xtraj, NX);
    printf("\n--- utraj ---\n");
    d_print_exp_tran_mat( NU, N, utraj, NU );*/
    // ocp_nlp_out_print(nlp_solver->dims, nlp_out);

    //printf("\nsolved ocp %d times, solution printed above\n\n", NTIMINGS);

    /*if (status == ACADOS_SUCCESS)
    {
        printf("UR5_acados_solve(): SUCCESS!\n");
    }
    else
    {
        printf("UR5_acados_solve() failed with status %d.\n", status);
    }*/

    // get solution
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);
    
    //***************************************
    //double results[18];
    results[15] = 0.0; // NaN marker
    /*for (int i=0;i<12;i++) {
        if (utraj[i]!=utraj[i]) results[15] = 0.0;
        else results[i] = utraj[i];
    }*/
    for (int i=0;i<6;i++) {
        if (xtraj[i+6]!=xtraj[i+6]) results[15] = 0.0; // x = [theta(6) omega(6)]
        else results[i] = xtraj[i+6];
        if (xtraj[i+6+12]!=xtraj[i+6+12]) results[15] = 0.0; // second horizon
        else results[i+6] = xtraj[i+6+NX];
    }
    
    results[12] = sqp_iter; results[13] = exec_time*1000; results[14] = kkt_norm_inf;
    //***************************************
    //for (int i=0;i<66;i++) trajectory[i] = xtraj[i];


    //UR5_acados_print_stats(acados_ocp_capsule);

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
