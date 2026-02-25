#include "acados_solver_holder.hpp"

my_NMPC_solver::my_NMPC_solver(int n) {
    num_steps = n; // set number of real-time iterations
    UR5_solver_capsule * my_acados_ocp_capsule = UR5_acados_create_capsule();
    acados_ocp_capsule = my_acados_ocp_capsule;
    // there is an opportunity to change the number of shooting intervals in C without new code generation
    int N = UR5_N;
    // allocate the array and fill it accordingly
    double* new_time_steps = NULL;
    int status = UR5_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);

    if (status) 
    {
        printf("UR5_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }
}

int my_NMPC_solver::solve_my_mpc(double current_joint_position[6],
    //  double current_human_position[56], 
     double current_joint_goal[6], double tracking_goal[60], double cgoal[3], double results[16], double my_weights[10], double full_trajectory[140]) {
    int status = -1;
    int N = UR5_N;

    ocp_nlp_config *nlp_config = UR5_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims *nlp_dims = UR5_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in *nlp_in = UR5_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out *nlp_out = UR5_acados_get_nlp_out(acados_ocp_capsule);
    ocp_nlp_solver *nlp_solver = UR5_acados_get_nlp_solver(acados_ocp_capsule);
    void *nlp_opts = UR5_acados_get_nlp_opts(acados_ocp_capsule);

    // initial condition
    int idxsh[NSH];
    for (int i=0;i<NSH;i++) idxsh[i]=i+3; // skiped first 3 constrains and set all other as slack

    double lh[NH];
    double uh[NH];
    for (int i=0;i<NH;i++) lh[i]=-10e6;
    for (int i=0;i<NH;i++) uh[i]=0.0;

    double lbx0[NBX0] = {0.0};
    double ubx0[NBX0] = {0.0};
    for (int i=0;i<6;i++) lbx0[i] = current_joint_position[i];
    for (int i=0;i<6;i++) ubx0[i] = current_joint_position[i];

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

    double lbxN[NBX0] = {0.0};
    double ubxN[NBX0] = {0.0};
    for (int i=0;i<6;i++) lbxN[i] = current_joint_goal[i];
    for (int i=0;i<6;i++) ubxN[i] = current_joint_goal[i];

    // initialization for state values
    double x_init[NX] = {0.0}; // set all zeros
    for (int j=0;j<6;j++) x_init[j]=current_joint_position[j]; // only 6 values in pose

    // initial value for control input
    double u0[NU]={0.0}; // set all zeros
    
    double W[(NU+NX)*(NU+NX)]= {0.0};
    double WN[(NX)*(NX)] = {0.0};

    for (int ii = 0; ii < (NX); ii++) W[ii+ii*(NU+NX)] = 100.0;
    for (int ii = NU; ii < (NU+NX); ii++) W[ii+ii*(NU+NX)] = 1.0;
    
    for (int ii = 0; ii < (NX); ii++) WN[ii+ii*(NX)] = 10*100;
    
    for (int ii=0;ii<N;ii++) {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "W", W);
    }
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", WN);

    double zl[NSH]={0.0};
    double zu[NSH]={0.0};

    double Zl[NSH] = {0.0};
    double Zu[NSH] = {0.0};
    for (int i=0;i<NSH;i++) Zl[i] = 0.0;
    for (int i=0;i<NSH;i++) Zu[i] = 10000.0;

    double y_ref[NY] = {0.0};
    for (int i=0;i<6;i++) y_ref[i] = current_joint_goal[i];
    double y_ref_N[NYN] = {0.0};
    for (int i=0;i<6;i++) y_ref_N[i] = current_joint_goal[i];

    for (int ii = 0; ii < N; ii++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "lh", lh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "uh", uh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "idxsh", idxsh);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "zl", zl);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "zu", zu);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "Zl", Zl);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "Zu", Zu);
    }
    
    // Set goal
    for (int ii=0;ii<N;ii++) {
        for (int i=0;i<6;i++) y_ref[i] = current_joint_goal[i]; 
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
        if (xtraj[i+6]!=xtraj[i+6]) results[15] = 1.0; // x = [theta(6) omega(6)]
        else results[i] = xtraj[i+6];
        if (xtraj[i+6+12]!=xtraj[i+6+12]) results[15] = 1.0; // second horizon
        else results[i+6] = xtraj[i+6+12];
    }
    
    results[12] = sqp_iter; results[13] = exec_time*1000; results[14] = ocp_cost;

    for (int j=0;j<20;j++) { // here we fix number of steps to pass to low-level controller to 10 instead of 20
        for (int i=0;i<6;i++) full_trajectory[7*j+i] = xtraj[NX*j+i];
        full_trajectory[7*j+6] = 0.250;// fixed dt; utraj[NU*j+6];
    }

    //***************************************
    //for (int i=0;i<66;i++) trajectory[i] = xtraj[i];


    //UR5_acados_print_stats(acados_ocp_capsule);

    //printf("\nSolver info:\n");
    /*printf(" SQP iterations %2d  minimum time for %d solve %f [ms]  cost %f \n",
           sqp_iter, NTIMINGS, exec_time*1000, ocp_cost);*/

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
