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
    init_07 test_05("test_05", preview_sampling_time_ms, nao.CoM_position[2], false);
    nao_igm nao_next = nao;

    
    vector<double> x_coord;
    vector<double> y_coord;
    vector<double> angle_rot;
    test_05.wmg->getFootsteps(x_coord, y_coord, angle_rot);


    smpc::solver solver(
        test_05.wmg->N, // size of the preview window
        400.0,  // Alpha
        4000.0,  // Beta
        1.0,    // Gamma
        0.01,   // regularization
        1e-7);  // tolerance
    //-----------------------------------------------------------

    //-----------------------------------------------------------
    test_05.par->init_state.set (nao.CoM_position[0], nao.CoM_position[1]);
    test_05.X_tilde.set (nao.CoM_position[0], nao.CoM_position[1]);
    //-----------------------------------------------------------


    initSDL();
    isRunning=1;
    test_05.wmg->T_ms[0] = control_sampling_time_ms;
    test_05.wmg->T_ms[1] = control_sampling_time_ms;
    while (isRunning)
    {
        nao.state_sensor = nao.state_model;
        if (next_preview_len_ms == 0)
        {
            next_preview_len_ms = preview_sampling_time_ms;
        }


        test_05.wmg->T_ms[2] = next_preview_len_ms;
        cout << test_05.wmg->isSupportSwitchNeeded() << endl;
        if (test_05.wmg->isSupportSwitchNeeded())
        {
            double pos_error[POSITION_VECTOR_SIZE];
            nao.switchSupportFoot(pos_error);
        }

        if (test_05.wmg->formPreviewWindow(*test_05.par) == WMG_HALT)
        {
            cout << "EXIT (halt = 1)" << endl;
            break;
        }


        //------------------------------------------------------
        solver.set_parameters (test_05.par->T, test_05.par->h, test_05.par->h[0], test_05.par->angle, test_05.par->zref_x, test_05.par->zref_y, test_05.par->lb, test_05.par->ub);
        solver.form_init_fp (test_05.par->fp_x, test_05.par->fp_y, test_05.par->init_state, test_05.par->X);
        solver.solve();
        //-----------------------------------------------------------
        // update state
        test_05.par->init_state.get_next_state (solver);
        //-----------------------------------------------------------


        //-----------------------------------------------------------
        // support foot and swing foot position/orientation
        double left_foot_pos[POSITION_VECTOR_SIZE + 1];
        double right_foot_pos[POSITION_VECTOR_SIZE + 1];


        test_05.wmg->getFeetPositions (
                control_sampling_time_ms,
                left_foot_pos,
                right_foot_pos);

        nao.setFeetPostures (left_foot_pos, right_foot_pos);

        // position of CoM
        nao.setCoM(test_05.par->init_state.x(), test_05.par->init_state.y(), test_05.par->hCoM);


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



        //-----------------------------------------------------------
        nao_next.state_model = nao.state_model;
        nao_next.support_foot = nao.support_foot;

        test_05.wmg->getFeetPositions (
                2*control_sampling_time_ms,
                left_foot_pos,
                right_foot_pos);

        nao_next.setFeetPostures (left_foot_pos, right_foot_pos);

        // position of CoM
        smpc::state_orig next_CoM;
        next_CoM.get_state(solver, 1);
        nao_next.setCoM(next_CoM.x(), next_CoM.y(), test_05.par->hCoM); 


        if (nao_next.igm () < 0)
        {
            cout << "IGM failed!" << endl;
            break;
        }
        failed_joint = nao_next.state_model.checkJointBounds();
        if (failed_joint >= 0)
        {
            cout << "MAX or MIN joint limit is violated! Number of the joint: " << failed_joint << endl;
            break;
        }
        //-----------------------------------------------------------
        drawSDL(100, x_coord, y_coord, angle_rot, nao_next.support_foot, nao_next.state_model.q, nao_next.support_foot_posture);

        next_preview_len_ms -= control_sampling_time_ms;
    }

    // keep the visualization active (until ESC is pressed)
    while (isRunning)
    {
        drawSDL(0, x_coord, y_coord, angle_rot, nao.support_foot, nao.state_model.q, nao.support_foot_posture);
    }

    return 0;
}
