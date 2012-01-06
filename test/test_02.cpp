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
#include "maple_functions.h"
#include "leg2joints.h"
#include "joints_sensors_id.h"


using namespace std;


#include "draw_SDL.cpp"
#include "init_steps_nao.cpp"


int main(int argc, char **argv)
{
    //-----------------------------------------------------------
    // sampling
    int control_sampling_time_ms = 10;
    int preview_sampling_time_ms = 100;
    int next_preview_len_ms = 0;
    //-----------------------------------------------------------

    //-----------------------------------------------------------
    // initialize classes
    WMG wmg;
    init_04 (&wmg);
    vector<double> x_coord;
    vector<double> y_coord;
    vector<double> angle_rot;
    wmg.getFootsteps(x_coord, y_coord, angle_rot);

    nao_igm nao;
    initNaoModel (&nao);
    wmg.init_param(
        (double) preview_sampling_time_ms / 1000, // sampling time in seconds
        nao.CoM_position[2],                      // height of the center of mass
        0.0135);

    smpc_solver solver(
        wmg.N, // size of the preview window
        300.0,  // Alpha
        800.0,  // Beta
        1.0,    // Gamma
        0.01,   // regularization
        1e-7);  // tolerance
    //-----------------------------------------------------------

    //-----------------------------------------------------------
    // initialize control & state matrices
    wmg.initABMatrices ((double) control_sampling_time_ms / 1000);
    wmg.init_state[0] = nao.CoM_position[0];
    wmg.init_state[1] = wmg.init_state[2] = 0;
    wmg.init_state[3] = nao.CoM_position[1];
    wmg.init_state[4] = wmg.init_state[5] = 0;
    wmg.next_control[0] = wmg.next_control[1] = 0;
    //-----------------------------------------------------------


    initSDL();
    isRunning=1;
    while (isRunning)
    {
        if (next_preview_len_ms == 0)
        {
            bool switch_foot = false;
            WMGret wmg_retval = wmg.FormPreviewWindow(&switch_foot);

            if (wmg_retval == WMG_HALT)
            {
                cout << "EXIT (halt = 1)" << endl;
                break;
            }

            if (switch_foot)
            {
                nao.switchSupportFoot();
            }

            next_preview_len_ms = preview_sampling_time_ms;
        }

        //------------------------------------------------------
        wmg.T[0] = (double) next_preview_len_ms / 1000; // get seconds
        solver.set_parameters (wmg.T, wmg.h, wmg.h[0], wmg.angle, wmg.zref_x, wmg.zref_y, wmg.lb, wmg.ub);
        solver.form_init_fp (wmg.fp_x, wmg.fp_y, wmg.init_state, wmg.X);
        solver.solve();
        //-----------------------------------------------------------
        // update state
        solver.get_first_controls (wmg.next_control);
        wmg.calculateNextState(wmg.next_control, wmg.init_state);
        //-----------------------------------------------------------

        //-----------------------------------------------------------
        // support foot and swing foot position/orientation
        double LegPos[POSITION_VECTOR_SIZE];
        double angle;
        wmg.getSwingFootPosition (
            WMG_SWING_2D_PARABOLA,
            1,
            1,
            LegPos,
            &angle);

        nao.initPosture (
            nao.swing_foot_posture,
            LegPos,
            0.0,    // roll angle
            0.0,    // pitch angle
            angle); // yaw angle

        // position of CoM
        nao.setCoM(wmg.init_state[0], wmg.init_state[3], wmg.hCoM);


        if (nao.igm_3(nao.swing_foot_posture, nao.CoM_position, nao.torso_orientation) < 0)
        {
            cout << "IGM failed!" << endl;
            break;
        }
        if (nao.checkJointBounds() >= 0)
        {
            cout << "MAX or MIN joint limit is violated!" << endl;
            break;
        }
        //-----------------------------------------------------------

        next_preview_len_ms -= control_sampling_time_ms;


        drawSDL(50, x_coord, y_coord, angle_rot, nao.support_foot, nao.q);
    }

    // keep the visualization active (until ESC is pressed)
    while (isRunning)
    {
        drawSDL(0, x_coord, y_coord, angle_rot, nao.support_foot, nao.q);
    }

    return 0;
}
