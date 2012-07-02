/** 
 * @file
 * @author Alexander Sherikov
 */

#include <iostream>
#include <fstream>
#include <cstdio>
#include <limits>
#include <cmath> // abs, M_PI
#include <cstring> //strcmp


#include "WMG.h"
#include "smpc_solver.h" 
#include "nao_igm.h" 
#include "joints_sensors_id.h"


using namespace std;


#include "init_steps_nao.cpp"
#include "tests_common.cpp"

void ik(init_11 &tdata, double CoMx, double CoMy, int time, int counter)
{
    tdata.wmg->getFeetPositions (
            time,
            tdata.nao.left_foot_posture.data(),
            tdata.nao.right_foot_posture.data());

    // position of CoM
    tdata.nao.setCoM(CoMx, CoMy, tdata.par->hCoM); 

    if (tdata.nao.igm(tdata.ref_angles, 1.2, 0.0015, 20) < 0)
    {
        printf("(%3i)  %s: IGM failed!\n", 
                counter,
                tdata.fs_out_filename.c_str());
    }
    int failed_joint = tdata.nao.state_model.checkJointBounds();
    if (failed_joint >= 0)
    {
        printf("(%3i)  %s: MAX or MIN joint limit is violated! Number of the joint: %i\n", 
                counter, 
                tdata.fs_out_filename.c_str(),
                failed_joint);
    }
}


int main(int argc, char **argv)
{
    //-----------------------------------------------------------
    // sampling
    int control_sampling_time_ms = 20;
    int preview_sampling_time_ms = 40;
    //-----------------------------------------------------------



    //-----------------------------------------------------------
    // initialize classes
    init_11 AS_test("test_10_as", preview_sampling_time_ms, false);


    smpc::solver_as AS_solver(
            AS_test.wmg->N, // size of the preview window
            8000.0,         // gain_position
            1.0,            // gain_velocity
            1.0,            // gain_acceleration
            1.0,            // gain_jerk
            1e-7,           // tolerance
            0,             // limit on the number of activated constraints
            true,           // enable constraint removal
            true);          // obj
    smpc::solver_as ASC_solver(
            AS_test.wmg->N, // size of the preview window
            8000.0,         // gain_position
            1.0,            // gain_velocity
            1.0,            // gain_acceleration
            1.0,            // gain_jerk
            1e-7,           // tolerance
            0,             // limit on the number of activated constraints
            false,           // enable constraint removal
            true);          // obj
    //-----------------------------------------------------------



    //-----------------------------------------------------------
    AS_test.nao.getCoM(AS_test.nao.state_sensor, AS_test.nao.CoM_position);
    AS_test.par->init_state.set (AS_test.nao.CoM_position[0], AS_test.nao.CoM_position[1]);
    //-----------------------------------------------------------



    AS_test.wmg->T_ms[0] = control_sampling_time_ms;
    AS_test.wmg->T_ms[1] = control_sampling_time_ms;
    int stop_iter = 79;
    smpc::state_com next_CoM;
    for(int counter = 0 ;; ++counter)
    {
// initialize
        if (AS_test.wmg->formPreviewWindow(*AS_test.par) == WMG_HALT)
        {
            cout << "EXIT (halt = 1)" << endl;
            break;
        }

// disturbance
        if (counter == stop_iter - 1)
        {
            AS_test.par->init_state.x() += 0.0;
            AS_test.par->init_state.x() -= 0.0;
            AS_test.par->init_state.y() += 0.0;
            AS_test.par->init_state.y() -= 0.015;
        }
        next_CoM = AS_test.par->init_state;
       
// solve QP        
        //------------------------------------------------------
        AS_solver.set_parameters (AS_test.par->T, AS_test.par->h, AS_test.par->h[0], AS_test.par->angle, AS_test.par->zref_x, AS_test.par->zref_y, AS_test.par->lb, AS_test.par->ub);
        AS_solver.form_init_fp (AS_test.par->fp_x, AS_test.par->fp_y, AS_test.par->init_state, AS_test.par->X);
        AS_solver.solve();
        AS_solver.get_next_state(AS_test.par->init_state);
        //-----------------------------------------------------------
        // update state
        printf("(%3i)  AS: AS size = %i // removed = %i // added = %i\n", 
                counter, 
                AS_solver.active_set_size, 
                AS_solver.removed_constraints_num,
                AS_solver.added_constraints_num);
        printf("           OBJ = ");
        for (unsigned int i = 0; i < AS_solver.objective_log.size(); ++i)
        {
            printf ("% 8e ", AS_solver.objective_log[i]);
        }
        printf("\n");
        //-----------------------------------------------------------
       
        //-----------------------------------------------------------
        ASC_solver.set_parameters (AS_test.par->T, AS_test.par->h, AS_test.par->h[0], AS_test.par->angle, AS_test.par->zref_x, AS_test.par->zref_y, AS_test.par->lb, AS_test.par->ub);
        ASC_solver.form_init_fp (AS_test.par->fp_x, AS_test.par->fp_y, next_CoM, AS_test.par->X);
        ASC_solver.solve();
        //-----------------------------------------------------------
        // update state
        printf("       ASC: AS size = %i // removed = %i // added = %i\n", 
                ASC_solver.active_set_size, 
                ASC_solver.removed_constraints_num,
                ASC_solver.added_constraints_num);
        printf("           OBJ = ");
        for (unsigned int i = 0; i < ASC_solver.objective_log.size(); ++i)
        {
            printf ("% 8e ", ASC_solver.objective_log[i]);
        }
        printf("\n ---\n");
        //-----------------------------------------------------------
    }
    return 0;
}

