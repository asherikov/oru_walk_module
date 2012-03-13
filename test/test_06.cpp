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
    //-----------------------------------------------------------

    //-----------------------------------------------------------
    // initialize classes
    nao_igm nao;
    double ref_angles[LOWER_JOINTS_NUM];
    initNaoModel (&nao, ref_angles);
    nao.getCoM(nao.state_sensor, nao.CoM_position);
//    init_09 test_06("test_06", preview_sampling_time_ms, nao.CoM_position[2], false);
    init_10 test_06("test_06", preview_sampling_time_ms, nao.CoM_position[2], false);

    
    vector<double> x_coord;
    vector<double> y_coord;
    vector<double> angle_rot;
    test_06.wmg->getFootsteps(x_coord, y_coord, angle_rot);


    smpc::solver solver(
        test_06.wmg->N, // size of the preview window
        1.0,  // Alpha
        4000.0,  // Beta
        1.0,    // Gamma
        0.01,   // regularization
        1e-7);  // tolerance
    //-----------------------------------------------------------

    //-----------------------------------------------------------
    test_06.par->init_state.set (nao.CoM_position[0], nao.CoM_position[1]);
    test_06.X_tilde.set (nao.CoM_position[0], nao.CoM_position[1]);
    //-----------------------------------------------------------


    initSDL();
    isRunning=1;
    test_06.wmg->T_ms[0] = control_sampling_time_ms;
    test_06.wmg->T_ms[1] = control_sampling_time_ms;
    while (isRunning)
    {
        nao.state_sensor = nao.state_model;

        if (test_06.wmg->formPreviewWindow(*test_06.par) == WMG_HALT)
        {
            cout << "EXIT (halt = 1)" << endl;
            break;
        }
        cout << test_06.wmg->isSupportSwitchNeeded() << endl;
        if (test_06.wmg->isSupportSwitchNeeded())
        {
            nao.switchSupportFoot();
        }


        //------------------------------------------------------
        solver.set_parameters (test_06.par->T, test_06.par->h, test_06.par->h[0], test_06.par->angle, test_06.par->zref_x, test_06.par->zref_y, test_06.par->lb, test_06.par->ub);
        solver.form_init_fp (test_06.par->fp_x, test_06.par->fp_y, test_06.par->init_state, test_06.par->X);
        solver.solve();
        //-----------------------------------------------------------
        // update state
        test_06.par->init_state.get_next_state (solver);
        //-----------------------------------------------------------


        //-----------------------------------------------------------
        // support foot and swing foot position/orientation
        test_06.wmg->getFeetPositions (
                control_sampling_time_ms,
                nao.left_foot_posture->data(),
                nao.right_foot_posture->data());

        // position of CoM
        nao.setCoM(test_06.par->init_state.x(), test_06.par->init_state.y(), test_06.par->hCoM);


        if (nao.igm(ref_angles, 1.2, 0.0015, 20) < 0)
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



        //-----------------------------------------------------------
        test_06.wmg->getFeetPositions (
                2*control_sampling_time_ms,
                nao.left_foot_posture->data(),
                nao.right_foot_posture->data());

        // position of CoM
        smpc::state_orig next_CoM;
        next_CoM.get_state(solver, 1);
        nao.setCoM(next_CoM.x(), next_CoM.y(), test_06.par->hCoM); 


        if (nao.igm(ref_angles, 1.2, 0.0015, 20) < 0)
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
        if (nao.support_foot == IGM_SUPPORT_RIGHT)
        {
            drawSDL(200, x_coord, y_coord, angle_rot, nao.support_foot, nao.state_model.q, *nao.right_foot_posture);
        }
        else
        {
            drawSDL(200, x_coord, y_coord, angle_rot, nao.support_foot, nao.state_model.q, *nao.left_foot_posture);
        }
    }

    // keep the visualization active (until ESC is pressed)
    while (isRunning)
    {
        if (nao.support_foot == IGM_SUPPORT_RIGHT)
        {
            drawSDL(0, x_coord, y_coord, angle_rot, nao.support_foot, nao.state_model.q, *nao.right_foot_posture);
        }
        else
        {
            drawSDL(0, x_coord, y_coord, angle_rot, nao.support_foot, nao.state_model.q, *nao.left_foot_posture);
        }
    }

    return 0;
}
