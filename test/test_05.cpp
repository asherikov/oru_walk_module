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
    init_08 tdata("test_05", preview_sampling_time_ms, false);

    
    vector<double> x_coord;
    vector<double> y_coord;
    vector<double> angle_rot;
    tdata.wmg->getFootsteps(x_coord, y_coord, angle_rot);


    smpc::solver_as solver(
        tdata.wmg->N, // size of the preview window
        4000.0,  // Beta
        1.0,  // Alpha
        0.02,   // regularization
        1.0,    // Gamma
        1e-7);  // tolerance
    //-----------------------------------------------------------

    //-----------------------------------------------------------
    tdata.nao.getCoM(tdata.nao.state_sensor, tdata.nao.CoM_position);
    tdata.par->init_state.set (tdata.nao.CoM_position[0], tdata.nao.CoM_position[1]);
    tdata.X_tilde.set (tdata.nao.CoM_position[0], tdata.nao.CoM_position[1]);
    //-----------------------------------------------------------


    initSDL();
    isRunning=1;
    tdata.wmg->T_ms[0] = control_sampling_time_ms;
    tdata.wmg->T_ms[1] = control_sampling_time_ms;
    while (isRunning)
    {
        tdata.nao.state_sensor = tdata.nao.state_model;

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


        //------------------------------------------------------
        solver.set_parameters (tdata.par->T, tdata.par->h, tdata.par->h[0], tdata.par->angle, tdata.par->zref_x, tdata.par->zref_y, tdata.par->lb, tdata.par->ub);
        solver.form_init_fp (tdata.par->fp_x, tdata.par->fp_y, tdata.par->init_state, tdata.par->X);
        solver.solve();
        //-----------------------------------------------------------
        // update state
        solver.get_next_state(tdata.par->init_state);
        //-----------------------------------------------------------


        //-----------------------------------------------------------
        // support foot and swing foot position/orientation
        tdata.wmg->getFeetPositions (
                control_sampling_time_ms,
                tdata.nao.left_foot_posture.data(),
                tdata.nao.right_foot_posture.data());

        // position of CoM
        tdata.nao.setCoM(tdata.par->init_state.x(), tdata.par->init_state.y(), tdata.par->hCoM);


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



        //-----------------------------------------------------------
        tdata.wmg->getFeetPositions (
                2*control_sampling_time_ms,
                tdata.nao.left_foot_posture.data(),
                tdata.nao.right_foot_posture.data());

        // position of CoM
        smpc::state_com next_CoM;
        solver.get_state (next_CoM, 1);
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
        if (tdata.nao.support_foot == IGM_SUPPORT_RIGHT)
        {
            drawSDL(0, x_coord, y_coord, angle_rot, tdata.nao.support_foot, tdata.nao.state_model.q, tdata.nao.right_foot_posture);
        }
        else
        {
            drawSDL(0, x_coord, y_coord, angle_rot, tdata.nao.support_foot, tdata.nao.state_model.q, tdata.nao.left_foot_posture);
        }
        usleep (100000);
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
