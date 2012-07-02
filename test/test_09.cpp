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

void ik(init_11 &tdata, double CoMx, double CoMy, int time, int counter, std::string solver)
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
                solver.c_str());
    }
    int failed_joint = tdata.nao.state_model.checkJointBounds();
    if (failed_joint >= 0)
    {
        printf("(%3i)  %s: MAX or MIN joint limit is violated! Number of the joint: %i\n", 
                counter,
                solver.c_str(),
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

    smpc::state_com next_CoM;
    smpc::solver_ip IP_solver(
            40,                      // N
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
            40,             // size of the preview window
            8000.0,         // gain_position
            1.0,            // gain_velocity
            1.0,            // gain_acceleration
            1.0,            // gain_jerk
            1e-7,           // tolerance
            0,             // limit on the number of activated constraints
            true,           // enable constraint removal
            true);          // obj
    
    for(int stop_iter = 1; stop_iter < 150; ++stop_iter)
    {
        printf("(%3i)=====================================\n", stop_iter);
        //-----------------------------------------------------------
        init_11 AS_test("", preview_sampling_time_ms, false);
        init_11 IP_test("", preview_sampling_time_ms, false);
        //-----------------------------------------------------------

        //-----------------------------------------------------------
        AS_test.nao.getCoM(AS_test.nao.state_sensor, AS_test.nao.CoM_position);
        AS_test.par->init_state.set (AS_test.nao.CoM_position[0], AS_test.nao.CoM_position[1]);
        AS_test.X_tilde.set (AS_test.nao.CoM_position[0], AS_test.nao.CoM_position[1]);
        IP_test.nao.getCoM(IP_test.nao.state_sensor, IP_test.nao.CoM_position);
        IP_test.par->init_state.set (IP_test.nao.CoM_position[0], IP_test.nao.CoM_position[1]);
        IP_test.X_tilde.set (IP_test.nao.CoM_position[0], IP_test.nao.CoM_position[1]);
        //-----------------------------------------------------------


        AS_test.wmg->T_ms[0] = control_sampling_time_ms;
        AS_test.wmg->T_ms[1] = control_sampling_time_ms;
        IP_test.wmg->T_ms[0] = control_sampling_time_ms;
        IP_test.wmg->T_ms[1] = control_sampling_time_ms;
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
            //-----------------------------------------------------------
           
            //-----------------------------------------------------------
            IP_solver.set_parameters (IP_test.par->T, IP_test.par->h, IP_test.par->h[0], IP_test.par->angle, IP_test.par->zref_x, IP_test.par->zref_y, IP_test.par->lb, IP_test.par->ub);
            IP_solver.form_init_fp (IP_test.par->fp_x, IP_test.par->fp_y, IP_test.par->init_state, IP_test.par->X);
            IP_solver.solve();
            //-----------------------------------------------------------
            // update state
            IP_solver.get_next_state(IP_test.par->init_state);
            //-----------------------------------------------------------
        

    // first IK
            //-----------------------------------------------------------
            ik(AS_test, AS_test.par->init_state.x(), AS_test.par->init_state.y(), control_sampling_time_ms, counter, "AS");
            ik(IP_test, IP_test.par->init_state.x(), IP_test.par->init_state.y(), control_sampling_time_ms, counter, "IP");
            //-----------------------------------------------------------

    // second IK
            //-----------------------------------------------------------
            AS_solver.get_state(next_CoM, 1);
            ik(AS_test, next_CoM.x(), next_CoM.y(), 2*control_sampling_time_ms, counter, "AS");
            IP_solver.get_state(next_CoM, 1);
            ik(IP_test, next_CoM.x(), next_CoM.y(), 2*control_sampling_time_ms, counter, "IP");
            //-----------------------------------------------------------
        }
    }

    return 0;
}

