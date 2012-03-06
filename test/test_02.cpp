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
    nao_igm nao;
    initNaoModel (&nao);
    init_08 test_02("test_02", preview_sampling_time_ms, nao.CoM_position[2], false);
    IPM ipm ((double) control_sampling_time_ms / 1000);

    
    vector<double> x_coord;
    vector<double> y_coord;
    vector<double> angle_rot;
    test_02.wmg->getFootsteps(x_coord, y_coord, angle_rot);


    smpc::solver solver(
        test_02.wmg->N, // size of the preview window
        400.0,  // Alpha
        4000.0,  // Beta
        1.0,    // Gamma
        0.01,   // regularization
        1e-7);  // tolerance
    //-----------------------------------------------------------

    //-----------------------------------------------------------
    test_02.par->init_state.set (nao.CoM_position[0], nao.CoM_position[1]);
    test_02.X_tilde.set (nao.CoM_position[0], nao.CoM_position[1]);
    //-----------------------------------------------------------


    initSDL();
    isRunning=1;
    while (isRunning)
    {
        nao.state_sensor = nao.state_model;
        if (next_preview_len_ms == 0)
        {
            if (test_02.wmg->formPreviewWindow(*test_02.par) == WMG_HALT)
            {
                cout << "EXIT (halt = 1)" << endl;
                break;
            }
            cout << test_02.wmg->isSupportSwitchNeeded() << endl;
            if (test_02.wmg->isSupportSwitchNeeded())
            {
                nao.switchSupportFoot();
            }

            next_preview_len_ms = preview_sampling_time_ms;
        }

        //------------------------------------------------------
        test_02.par->T[0] = (double) next_preview_len_ms / 1000; // get seconds
        solver.set_parameters (test_02.par->T, test_02.par->h, test_02.par->h[0], test_02.par->angle, test_02.par->zref_x, test_02.par->zref_y, test_02.par->lb, test_02.par->ub);
        solver.form_init_fp (test_02.par->fp_x, test_02.par->fp_y, test_02.par->init_state, test_02.par->X);
        solver.solve();
        //-----------------------------------------------------------
        // update state
        ipm.control_vector.get_first_controls (solver);
        ipm.calculateNextState(ipm.control_vector, test_02.par->init_state);
        //-----------------------------------------------------------


        //-----------------------------------------------------------
        // support foot and swing foot position/orientation
        test_02.wmg->getFeetPositions (
                preview_sampling_time_ms,
                nao.left_foot_posture.data(),
                nao.right_foot_posture.data());

        // position of CoM
        smpc::state_orig next_CoM;
        next_CoM.get_state(solver, 0);
        nao.setCoM(next_CoM.x(), next_CoM.y(), test_02.par->hCoM); 


        if (nao.igm () < 0)
        {
            cout << "IGM failed!" << endl;
            break;
        }
        int failed_joint = nao.state_model.checkJointBounds();
        if (failed_joint >= 0)
        {
            cout << "MAX or MIN joint limit is violated! Number of the joint: " << failed_joint << endl;
            break;
        }
        //-----------------------------------------------------------


        if (nao.support_foot == IGM_SUPPORT_RIGHT)
        {
            drawSDL(50, x_coord, y_coord, angle_rot, nao.support_foot, nao.state_model.q, nao.right_foot_posture);
        }
        else
        {
            drawSDL(50, x_coord, y_coord, angle_rot, nao.support_foot, nao.state_model.q, nao.left_foot_posture);
        }



        //-----------------------------------------------------------
        test_02.wmg->getFeetPositions (
                2*preview_sampling_time_ms,
                nao.left_foot_posture.data(),
                nao.right_foot_posture.data());


        // position of CoM
        next_CoM.get_state(solver, 1);
        nao.setCoM(next_CoM.x(), next_CoM.y(), test_02.par->hCoM); 


        if (nao.igm () < 0)
        {
            cout << "IGM failed!" << endl;
            break;
        }
        failed_joint = nao.state_model.checkJointBounds();
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
        if (nao.support_foot == IGM_SUPPORT_RIGHT)
        {
            drawSDL(0, x_coord, y_coord, angle_rot, nao.support_foot, nao.state_model.q, nao.right_foot_posture);
        }
        else
        {
            drawSDL(0, x_coord, y_coord, angle_rot, nao.support_foot, nao.state_model.q, nao.left_foot_posture);
        }
    }

    return 0;
}
