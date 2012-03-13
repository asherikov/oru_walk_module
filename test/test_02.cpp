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
#include "leg2joints.h"
#include "joints_sensors_id.h"


using namespace std;


#include "draw_SDL.cpp"
#include "init_steps_nao.cpp"


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
    init_08 tdata("test_02", preview_sampling_time_ms, false);
    IPM ipm ((double) control_sampling_time_ms / 1000);

    
    vector<double> x_coord;
    vector<double> y_coord;
    vector<double> angle_rot;
    tdata.wmg->getFootsteps(x_coord, y_coord, angle_rot);


    smpc::solver solver(
        tdata.wmg->N, // size of the preview window
        400.0,  // Alpha
        4000.0,  // Beta
        1.0,    // Gamma
        0.01,   // regularization
        1e-7);  // tolerance
    //-----------------------------------------------------------

    //-----------------------------------------------------------
    tdata.nao.getCoM(tdata.nao.state_sensor, tdata.nao.CoM_position);
    tdata.par->init_state.set (tdata.nao.CoM_position[0], tdata.nao.CoM_position[1]);
    tdata.X_tilde.set (tdata.nao.CoM_position[0], tdata.nao.CoM_position[1]);
    //-----------------------------------------------------------


    initSDL();
    isRunning=1;
    while (isRunning)
    {
        tdata.nao.state_sensor = tdata.nao.state_model;
        if (next_preview_len_ms == 0)
        {
            if (tdata.wmg->formPreviewWindow(*tdata.par) == WMG_HALT)
            {
                cout << "EXIT (halt = 1)" << endl;
                break;
            }
            cout << tdata.wmg->isSupportSwitchNeeded() << endl;
            if (tdata.wmg->isSupportSwitchNeeded())
            {
                tdata.nao.switchSupportFoot();
            }

            next_preview_len_ms = preview_sampling_time_ms;
        }

        //------------------------------------------------------
        tdata.par->T[0] = (double) next_preview_len_ms / 1000; // get seconds
        solver.set_parameters (tdata.par->T, tdata.par->h, tdata.par->h[0], tdata.par->angle, tdata.par->zref_x, tdata.par->zref_y, tdata.par->lb, tdata.par->ub);
        solver.form_init_fp (tdata.par->fp_x, tdata.par->fp_y, tdata.par->init_state, tdata.par->X);
        solver.solve();
        //-----------------------------------------------------------
        // update state
        ipm.control_vector.get_first_controls (solver);
        ipm.calculateNextState(ipm.control_vector, tdata.par->init_state);
        //-----------------------------------------------------------


        //-----------------------------------------------------------
        // support foot and swing foot position/orientation
        tdata.wmg->getFeetPositions (
                preview_sampling_time_ms,
                tdata.nao.left_foot_posture.data(),
                tdata.nao.right_foot_posture.data());

        // position of CoM
        smpc::state_orig next_CoM;
        next_CoM.get_state(solver, 0);
        tdata.nao.setCoM(next_CoM.x(), next_CoM.y(), tdata.par->hCoM); 


        if (tdata.nao.igm(tdata.ref_angles, 1.2, 0.0015, 20) < 0)
        {
            cout << "IGM failed!" << endl;
            break;
        }
        int failed_joint = tdata.nao.state_model.checkJointBounds();
        if (failed_joint >= 0)
        {
            cout << "MAX or MIN joint limit is violated! Number of the joint: " << failed_joint << endl;
            break;
        }
        //-----------------------------------------------------------


        if (tdata.nao.support_foot == IGM_SUPPORT_RIGHT)
        {
            drawSDL(50, x_coord, y_coord, angle_rot, tdata.nao.support_foot, tdata.nao.state_model.q, tdata.nao.right_foot_posture);
        }
        else
        {
            drawSDL(50, x_coord, y_coord, angle_rot, tdata.nao.support_foot, tdata.nao.state_model.q, tdata.nao.left_foot_posture);
        }



        //-----------------------------------------------------------
        tdata.wmg->getFeetPositions (
                2*preview_sampling_time_ms,
                tdata.nao.left_foot_posture.data(),
                tdata.nao.right_foot_posture.data());


        // position of CoM
        next_CoM.get_state(solver, 1);
        tdata.nao.setCoM(next_CoM.x(), next_CoM.y(), tdata.par->hCoM); 


        if (tdata.nao.igm(tdata.ref_angles, 1.2, 0.0015, 20) < 0)
        {
            cout << "IGM failed!" << endl;
            break;
        }
        failed_joint = tdata.nao.state_model.checkJointBounds();
        if (failed_joint >= 0)
        {
            cout << "MAX or MIN joint limit is violated! Number of the joint: " << failed_joint << endl;
            break;
        }
        //-----------------------------------------------------------

        next_preview_len_ms -= control_sampling_time_ms;
    }

    // keep the visualization active (until ESC is pressed)
    while (isRunning)
    {
        if (tdata.nao.support_foot == IGM_SUPPORT_RIGHT)
        {
            drawSDL(0, x_coord, y_coord, angle_rot, tdata.nao.support_foot, tdata.nao.state_model.q, tdata.nao.right_foot_posture);
        }
        else
        {
            drawSDL(0, x_coord, y_coord, angle_rot, tdata.nao.support_foot, tdata.nao.state_model.q, tdata.nao.left_foot_posture);
        }
    }

    return 0;
}
