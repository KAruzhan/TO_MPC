/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */


// standard
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/sim_interface.h"
#include "acados_sim_solver_UR5.h"

#define NX     UR5_NX
#define NZ     UR5_NZ
#define NU     UR5_NU
#define NP     UR5_NP


int main()
{
    int status = 0;
    UR5_sim_solver_capsule *capsule = UR5_acados_sim_solver_create_capsule();
    status = UR5_acados_sim_create(capsule);

    if (status)
    {
        printf("acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    sim_config *acados_sim_config = UR5_acados_get_sim_config(capsule);
    sim_in *acados_sim_in = UR5_acados_get_sim_in(capsule);
    sim_out *acados_sim_out = UR5_acados_get_sim_out(capsule);
    void *acados_sim_dims = UR5_acados_get_sim_dims(capsule);

    // initial condition
    double x_current[NX];
    x_current[0] = 0.0;
    x_current[1] = 0.0;
    x_current[2] = 0.0;
    x_current[3] = 0.0;
    x_current[4] = 0.0;
    x_current[5] = 0.0;
    x_current[6] = 0.0;
    x_current[7] = 0.0;
    x_current[8] = 0.0;
    x_current[9] = 0.0;
    x_current[10] = 0.0;
    x_current[11] = 0.0;

  
    x_current[0] = 0;
    x_current[1] = -2.3;
    x_current[2] = -1.1;
    x_current[3] = -1.2;
    x_current[4] = -1.2;
    x_current[5] = 0.5;
    
    printf("main_sim: NOTE: initial state not fully defined via lbx_0, using 0.0 for indices that are not in idxbx_0.");
  


    // initial value for control input
    double u0[NU];
    u0[0] = 0.0;
    u0[1] = 0.0;
    u0[2] = 0.0;
    u0[3] = 0.0;
    u0[4] = 0.0;
    u0[5] = 0.0;
    // set parameters
    double p[NP];
    p[0] = 100;
    p[1] = -0.5;
    p[2] = 0.45;
    p[3] = 0.15;
    p[4] = 100.0658;
    p[5] = 0.4526;
    p[6] = 0.8624;
    p[7] = 0.25;
    p[8] = 100.0844;
    p[9] = 0.7044;
    p[10] = 0.9207;
    p[11] = 0.15;
    p[12] = 100.2083;
    p[13] = 0.3075;
    p[14] = 1.0208;
    p[15] = 0.15;
    p[16] = 100.0556;
    p[17] = 0.6289;
    p[18] = 0.7595;
    p[19] = 0.15;
    p[20] = 100.2024;
    p[21] = 0.2732;
    p[22] = 0.8478;
    p[23] = 0.15;
    p[24] = 100.0267;
    p[25] = 0.5535;
    p[26] = 0.5983;
    p[27] = 0.15;
    p[28] = 100.1965;
    p[29] = 0.2389;
    p[30] = 0.6749;
    p[31] = 0.15;
    p[32] = -100.0208;
    p[33] = 0.3964;
    p[34] = 0.5857;
    p[35] = 0.1;
    p[36] = 100.0546;
    p[37] = 0.2951;
    p[38] = 0.6132;
    p[39] = 0.1;
    p[40] = -100.1062;
    p[41] = 0.2444;
    p[42] = 0.5897;
    p[43] = 0.13;
    p[44] = -100.0998;
    p[45] = 0.3062;
    p[46] = 0.5387;
    p[47] = 0.13;
    p[48] = 100.1908;
    p[49] = 0.529;
    p[50] = 1.0016;
    p[51] = 0.2;
    p[52] = 100.2106;
    p[53] = 0.4602;
    p[54] = 0.6915;
    p[55] = 0.25;
    p[56] = 0.4824;
    p[57] = -0.1737;
    p[58] = 0.4643;

    UR5_acados_sim_update_params(capsule, p, NP);
  

  


    int n_sim_steps = 3;
    // solve ocp in loop
    for (int ii = 0; ii < n_sim_steps; ii++)
    {
        // set inputs
        sim_in_set(acados_sim_config, acados_sim_dims,
            acados_sim_in, "x", x_current);
        sim_in_set(acados_sim_config, acados_sim_dims,
            acados_sim_in, "u", u0);

        // solve
        status = UR5_acados_sim_solve(capsule);
        if (status != ACADOS_SUCCESS)
        {
            printf("acados_solve() failed with status %d.\n", status);
        }

        // get outputs
        sim_out_get(acados_sim_config, acados_sim_dims,
               acados_sim_out, "x", x_current);

    

        // print solution
        printf("\nx_current, %d\n", ii);
        for (int jj = 0; jj < NX; jj++)
        {
            printf("%e\n", x_current[jj]);
        }
    }

    printf("\nPerformed %d simulation steps with acados integrator successfully.\n\n", n_sim_steps);

    // free solver
    status = UR5_acados_sim_free(capsule);
    if (status) {
        printf("UR5_acados_sim_free() returned status %d. \n", status);
    }

    UR5_acados_sim_solver_free_capsule(capsule);

    return status;
}
