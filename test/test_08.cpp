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
    int next_preview_len_ms = 0;
    //-----------------------------------------------------------



    //-----------------------------------------------------------
    // initialize classes
    init_11 AS_test("test_08_as", preview_sampling_time_ms, false);
    init_11 IP_test("test_08_ip", preview_sampling_time_ms, false);

    smpc::solver_ip IP_solver(
            IP_test.wmg->N,          // N
            8000,                    // gain_position
            1.0,                     // gain_velocity
            1.0,                     // gain_acceleration
            1.0,                     // gain_jerk
            1e-1,                    // tol
            10000,                   // tol_out | if it is big only one iteration is made.
            1e-1,                    // t
            1,                       // mu
            0.01,                    // bs_alpha
            0.90,                    // bs_beta
            3,                       // max_iter
            smpc::SMPC_IP_BS_LOGBAR, // backtracking search
            true);                   // obj


    smpc::solver_as AS_solver(
            AS_test.wmg->N, // size of the preview window
            8000.0,         // gain_position
            1.0,            // gain_velocity
            1.0,            // gain_acceleration
            1.0,            // gain_jerk
            1e-7,           // tolerance
            30,             // limit on the number of activated constraints
            true,           // enable constraint removal
            true);          // obj
    //-----------------------------------------------------------



    //-----------------------------------------------------------
    AS_test.nao.getCoM(AS_test.nao.state_sensor, AS_test.nao.CoM_position);
    AS_test.par->init_state.set (AS_test.nao.CoM_position[0], AS_test.nao.CoM_position[1]);
    AS_test.X_tilde.set (AS_test.nao.CoM_position[0], AS_test.nao.CoM_position[1]);
    IP_test.nao.getCoM(IP_test.nao.state_sensor, IP_test.nao.CoM_position);
    IP_test.par->init_state.set (IP_test.nao.CoM_position[0], IP_test.nao.CoM_position[1]);
    IP_test.X_tilde.set (IP_test.nao.CoM_position[0], IP_test.nao.CoM_position[1]);
    //-----------------------------------------------------------



    //-----------------------------------------------------------
    // output
    test_log AS_log(AS_test.fs_out_filename.c_str());
    test_log IP_log(IP_test.fs_out_filename.c_str());
    //-----------------------------------------------------------




    AS_test.wmg->T_ms[0] = control_sampling_time_ms;
    AS_test.wmg->T_ms[1] = control_sampling_time_ms;
    IP_test.wmg->T_ms[0] = control_sampling_time_ms;
    IP_test.wmg->T_ms[1] = control_sampling_time_ms;
    int stop_iter = 79;
    smpc::state_com next_CoM;
    for(int counter = 0 ;; ++counter)
    {
// initialize
        AS_test.nao.state_sensor = AS_test.nao.state_model;
        IP_test.nao.state_sensor = IP_test.nao.state_model;


        if (AS_test.wmg->formPreviewWindow(*AS_test.par) == WMG_HALT)
        {
            cout << "EXIT (halt = 1)" << endl;
            break;
        }
        if (IP_test.wmg->formPreviewWindow(*IP_test.par) == WMG_HALT)
        {
            cout << "EXIT (halt = 1)" << endl;
            break;
        }
        for (unsigned int j = 0; j < AS_test.wmg->N; j++)
        {
            AS_log.addZMPrefPoint(AS_test.par->zref_x[j], AS_test.par->zref_y[j]);
            IP_log.addZMPrefPoint(IP_test.par->zref_x[j], IP_test.par->zref_y[j]);
        }
        if (AS_test.wmg->isSupportSwitchNeeded())
        {
            AS_test.nao.switchSupportFoot();
            IP_test.nao.switchSupportFoot();
        }

// disturbance
        if (counter == stop_iter - 1)
        {
            IP_test.par->init_state.x() += 0.0;
            IP_test.par->init_state.x() -= 0.0;
            IP_test.par->init_state.y() += 0.0;
            IP_test.par->init_state.y() -= 0.015;

            AS_test.par->init_state.x() += 0.0;
            AS_test.par->init_state.x() -= 0.0;
            AS_test.par->init_state.y() += 0.0;
            AS_test.par->init_state.y() -= 0.015;
        }
       
// solve QP        
        //------------------------------------------------------
        AS_solver.set_parameters (AS_test.par->T, AS_test.par->h, AS_test.par->h[0], AS_test.par->angle, AS_test.par->zref_x, AS_test.par->zref_y, AS_test.par->lb, AS_test.par->ub);
        AS_solver.form_init_fp (AS_test.par->fp_x, AS_test.par->fp_y, AS_test.par->init_state, AS_test.par->X);
        AS_solver.solve();
        //-----------------------------------------------------------
        // update state
        AS_solver.get_next_state(AS_test.par->init_state);
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
        IP_solver.set_parameters (IP_test.par->T, IP_test.par->h, IP_test.par->h[0], IP_test.par->angle, IP_test.par->zref_x, IP_test.par->zref_y, IP_test.par->lb, IP_test.par->ub);
        IP_solver.form_init_fp (IP_test.par->fp_x, IP_test.par->fp_y, IP_test.par->init_state, IP_test.par->X);
        IP_solver.solve();
        //-----------------------------------------------------------
        // update state
        IP_solver.get_next_state(IP_test.par->init_state);
        printf("       IP: ext = %d // int = %d // bs = %d\n",
                IP_solver.ext_loop_iterations,
                IP_solver.int_loop_iterations,
                IP_solver.bt_search_iterations);
        printf("           OBJ = ");
        for (unsigned int i = 0; i < IP_solver.objective_log.size(); ++i)
        {
            printf ("% 8e ", IP_solver.objective_log[i]);
        }
        printf("\n ---\n");
        //-----------------------------------------------------------



// log ZMP, CoM
        //-----------------------------------------------------------
        // output
        if (next_preview_len_ms == 0)
        {
            next_preview_len_ms = preview_sampling_time_ms;

            AS_log.addZMPpoint (AS_test.X_tilde.x(), AS_test.X_tilde.y());
            AS_solver.get_next_state(AS_test.X_tilde);
            IP_log.addZMPpoint (IP_test.X_tilde.x(), IP_test.X_tilde.y());
            IP_solver.get_next_state(IP_test.X_tilde);
        }
        AS_log.addCoMpoint (AS_test.par->init_state.x(), AS_test.par->init_state.y());
        IP_log.addCoMpoint (IP_test.par->init_state.x(), IP_test.par->init_state.y());

        next_preview_len_ms -= control_sampling_time_ms;
        //-----------------------------------------------------------
    

// first IK
        //-----------------------------------------------------------
        ik(AS_test, AS_test.par->init_state.x(), AS_test.par->init_state.y(), control_sampling_time_ms, counter);
        ik(IP_test, IP_test.par->init_state.x(), IP_test.par->init_state.y(), control_sampling_time_ms, counter);
        //-----------------------------------------------------------

// log feet
        //-----------------------------------------------------------
        // output
        AS_log.addFeetPositions (AS_test.nao);
        IP_log.addFeetPositions (IP_test.nao);
        //-----------------------------------------------------------
       
// second IK
        //-----------------------------------------------------------
        AS_solver.get_state(next_CoM, 1);
        ik(AS_test, next_CoM.x(), next_CoM.y(), 2*control_sampling_time_ms, counter);
        IP_solver.get_state(next_CoM, 1);
        ik(IP_test, next_CoM.x(), next_CoM.y(), 2*control_sampling_time_ms, counter);
        //-----------------------------------------------------------
    }



    //-----------------------------------------------------------
    // output
//    AS_log.flushLeftFoot ();
//    AS_log.flushRightFoot ();
    AS_log.flushZMP ();
    AS_log.flushZMPref ();
    AS_log.flushCoM ();
    IP_log.flushZMP ();
    IP_log.flushZMPref ();
    IP_log.flushCoM ();
    //-----------------------------------------------------------

    return 0;
}

