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



int main(int argc, char **argv)
{
    //-----------------------------------------------------------
    // sampling
    int control_sampling_time_ms = 10;
    int preview_sampling_time_ms = 20;
    int next_preview_len_ms = 0;
    //-----------------------------------------------------------



    //-----------------------------------------------------------
    // initialize classes
    WMG wmg;
    init_07 (&wmg);


    nao_igm nao;
    initNaoModel (&nao);
    wmg.init_param(
            (double) preview_sampling_time_ms / 1000, // sampling time in seconds
            nao.CoM_position[2],                      // height of the center of mass
            0.015);

    std::string fs_out_filename("test_03_fs.m");
    wmg.FS2file(fs_out_filename, false); // output results for later use in Matlab/Octave


    smpc_solver solver(
            wmg.N, // size of the preview window
            1500.0,  // Alpha
            9000.0,  // Beta
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



    //-----------------------------------------------------------
    // output
    FILE *file_op = fopen(fs_out_filename.c_str(), "a");
    fprintf(file_op,"hold on\n");

    vector<double> ZMP_x;
    vector<double> ZMP_y;
    vector<double> ZMPref_x;
    vector<double> ZMPref_y;
    vector<double> CoM_x;
    vector<double> CoM_y;

    vector<double> swing_foot_x;
    vector<double> swing_foot_y;
    vector<double> swing_foot_z;
    //-----------------------------------------------------------





    for(int i=0 ;; i++)
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
            for (int j = 0; j < wmg.N; j++)
            {
                ZMPref_x.push_back(wmg.zref_x[j]);
                ZMPref_y.push_back(wmg.zref_y[j]);
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
        // output
        if (next_preview_len_ms == preview_sampling_time_ms)
        {
            solver.get_next_state_tilde (wmg.X_tilde);
            ZMP_x.push_back(wmg.X_tilde[0]);
            ZMP_y.push_back(wmg.X_tilde[3]);
        }
        CoM_x.push_back(wmg.init_state[0]);
        CoM_y.push_back(wmg.init_state[3]);
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
        int failed_joint = nao.checkJointBounds();
        if (failed_joint >= 0)
        {
            cout << "MAX or MIN joint limit is violated! Number of the joint: " << failed_joint << endl;
            break;
        }
        //-----------------------------------------------------------



        //-----------------------------------------------------------
        // output
        swing_foot_x.push_back(LegPos[0]);
        swing_foot_y.push_back(LegPos[1]);
        swing_foot_z.push_back(LegPos[2]);
        //-----------------------------------------------------------
        


        next_preview_len_ms -= control_sampling_time_ms;
    }



    //-----------------------------------------------------------
    // output
    //printVectors (file_op, swing_foot_x, swing_foot_y, swing_foot_z, "SFP", "r");
    printVectors (file_op, ZMP_x, ZMP_y, "ZMP", "k");
    printVectors (file_op, ZMPref_x, ZMPref_y, "ZMPref", "x");
    printVectors (file_op, CoM_x, CoM_y, "CoM", "b");
    fprintf(file_op,"hold off\n");
    fclose(file_op);
    //-----------------------------------------------------------

    return 0;
}

