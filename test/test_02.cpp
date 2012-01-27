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

    smpc::solver solver(
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
    wmg.init_state.set (nao.CoM_position[0], nao.CoM_position[1]);
    wmg.X_tilde.set (nao.CoM_position[0], nao.CoM_position[1]);
    //-----------------------------------------------------------


    initSDL();
    isRunning=1;
    while (isRunning)
    {
        if (next_preview_len_ms == 0)
        {
            if (wmg.isSupportSwitchNeeded())
            {
                double pos_error[POSITION_VECTOR_SIZE];
                nao.switchSupportFoot(pos_error);
            }

            WMGret wmg_retval = wmg.formPreviewWindow();

            if (wmg_retval == WMG_HALT)
            {
                cout << "EXIT (halt = 1)" << endl;
                break;
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
        wmg.next_control.get_first_controls (solver);
        wmg.calculateNextState(wmg.next_control, wmg.init_state);
        //-----------------------------------------------------------

        //-----------------------------------------------------------
        // support foot and swing foot position/orientation
        double left_foot_pos[POSITION_VECTOR_SIZE + 1];
        double right_foot_pos[POSITION_VECTOR_SIZE + 1];
        wmg.getFeetPositions (
                preview_sampling_time_ms/control_sampling_time_ms,
                (preview_sampling_time_ms - next_preview_len_ms)/control_sampling_time_ms+1,
                left_foot_pos,
                right_foot_pos);

        nao.setFeetPostures (left_foot_pos, right_foot_pos);

        // position of CoM
        nao.setCoM(wmg.init_state.x(), wmg.init_state.y(), wmg.hCoM); 


        if (nao.igm () < 0)
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


        drawSDL(50, x_coord, y_coord, angle_rot, nao.state.support_foot, nao.state.q);
    }

    // keep the visualization active (until ESC is pressed)
    while (isRunning)
    {
        drawSDL(0, x_coord, y_coord, angle_rot, nao.state.support_foot, nao.state.q);
    }

    return 0;
}
