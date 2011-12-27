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


/**
 * @brief Walk straight
 */
void init_04 (WMG *wmg)
{
    double d[4];
    wmg->init(15);

    // each step is defined relatively to the previous step
    double step_x = 0.035;      // relative X position
    double step_y = 0.1;       // relative Y position


    d[0] = 0.09;
    d[1] = 0.025;
    d[2] = 0.03;
    d[3] = 0.025;
    wmg->AddFootstep(0.0, step_y/2, 0.0, 0, 0, d, FS_TYPE_SS_L);

    // Initial double support
    d[0] = 0.09;
    d[1] = 0.075;
    d[2] = 0.03;
    d[3] = 0.075;
    wmg->AddFootstep(0.0, -step_y/2, 0.0, 1, 1, d, FS_TYPE_DS);
    // ZMP, CoM are at [0;0]


    // all subsequent steps have normal feet size
    d[0] = 0.09;
    d[1] = 0.025;
    d[2] = 0.03;
    d[3] = 0.025;
    // 2 reference ZMP positions in single support 
    // 1 in double support
    // 1 + 2 = 3
    wmg->AddFootstep(0.0   , -step_y/2, 0.0 , 4,  6, d);
    wmg->AddFootstep(step_x,  step_y, 0.0);
    wmg->AddFootstep(step_x, -step_y, 0.0);
    wmg->AddFootstep(step_x,  step_y, 0.0);
    wmg->AddFootstep(step_x, -step_y, 0.0);
    wmg->AddFootstep(step_x,  step_y, 0.0);

    // here we give many reference points, since otherwise we 
    // would not have enough steps in preview window to reach 
    // the last footsteps
    d[0] = 0.09;
    d[1] = 0.075;
    d[2] = 0.03;
    d[3] = 0.075;
    wmg->AddFootstep(0.0   , -step_y/2, 0.0, 30, 30, d, FS_TYPE_DS);
    d[0] = 0.09;
    d[1] = 0.025;
    d[2] = 0.03;
    d[3] = 0.025;
    wmg->AddFootstep(0.0   , -step_y/2, 0.0 , 0,  0, d, FS_TYPE_SS_R);
}



void initNaoModel (nao_igm* nao)
{
    // joint angles
    nao->q[L_HIP_YAW_PITCH]  =  0.0;
    nao->q[R_HIP_YAW_PITCH]  =  0.0;

    nao->q[L_HIP_ROLL]       = -0.000384;
    nao->q[L_HIP_PITCH]      = -0.598291;
    nao->q[L_KNEE_PITCH]     =  1.009413;
    nao->q[L_ANKLE_PITCH]    = -0.492352;
    nao->q[L_ANKLE_ROLL]     =  0.000469;

    nao->q[R_HIP_ROLL]       = -0.000384;
    nao->q[R_HIP_PITCH]      = -0.598219;
    nao->q[R_KNEE_PITCH]     =  1.009237;
    nao->q[R_ANKLE_PITCH]    = -0.492248;
    nao->q[R_ANKLE_ROLL]     =  0.000469;

    nao->q[L_SHOULDER_PITCH] =  1.418908;
    nao->q[L_SHOULDER_ROLL]  =  0.332836;
    nao->q[L_ELBOW_YAW]      = -1.379108;
    nao->q[L_ELBOW_ROLL]     = -1.021602;
    nao->q[L_WRIST_YAW]      = -0.013848;

    nao->q[R_SHOULDER_PITCH] =  1.425128;
    nao->q[R_SHOULDER_ROLL]  = -0.331386;
    nao->q[R_ELBOW_YAW]      =  1.383626;
    nao->q[R_ELBOW_ROLL]     =  1.029356;
    nao->q[R_WRIST_YAW]      = -0.01078;

    nao->q[HEAD_PITCH]       =  0.0;
    nao->q[HEAD_YAW]         =  0.0;



    // support foot position and orientation
    /// @attention Hardcoded parameters.
    double foot_position[POSITION_VECTOR_SIZE] = {0.0, -0.05, 0.0};
    double foot_orientation[ORIENTATION_MATRIX_SIZE] = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0};
    nao->init (
            IGM_SUPPORT_RIGHT,
            foot_position,
            foot_orientation);
}




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


    nao_igm nao;
    initNaoModel (&nao);
    wmg.init_param(
            (double) preview_sampling_time_ms / 1000, // sampling time in seconds
            nao.CoM_position[2],                      // height of the center of mass
            0.0135);

    std::string fs_out_filename("test_01_fs.m");
    wmg.FS2file(fs_out_filename); // output results for later use in Matlab/Octave


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
    wmg.initState (nao.CoM_position[0], nao.CoM_position[1], wmg.X_tilde);
    double cur_control[2];
    cur_control[0] = cur_control[1] = 0;
    //-----------------------------------------------------------



    //-----------------------------------------------------------
    // output
    FILE *file_op = fopen(fs_out_filename.c_str(), "a");
    fprintf(file_op,"hold on\n");

    vector<double> ZMP_x;
    vector<double> ZMP_y;
    vector<double> CoM_x;
    vector<double> CoM_y;

    vector<double> swing_foot_x;
    vector<double> swing_foot_y;
    vector<double> swing_foot_z;
    //-----------------------------------------------------------




    double X[SMPC_NUM_STATE_VAR];

    for(int i=0 ;; i++)
    {
        if (next_preview_len_ms == 0)
        {
            WMGret wmg_retval = wmg.FormPreviewWindow();

            if (wmg_retval == WMG_HALT)
            {
                cout << "EXIT (halt = 1)" << endl;
                break;
            }

            if (wmg_retval == WMG_SWITCH_REFERENCE_FOOT)
            {
                nao.switchSupportFoot();
            }

            next_preview_len_ms = preview_sampling_time_ms;
        }   

       
        
        //------------------------------------------------------
        wmg.T[0] = (double) next_preview_len_ms / 1000; // get seconds
        solver.set_parameters (wmg.T, wmg.h, wmg.angle, wmg.zref_x, wmg.zref_y, wmg.lb, wmg.ub);
        solver.form_init_fp (wmg.fp_x, wmg.fp_y, wmg.X_tilde, wmg.X);
        solver.solve();
        solver.get_next_state (X);
        solver.get_first_controls (cur_control);
        //------------------------------------------------------


        //-----------------------------------------------------------
        // update state
        wmg.calculateNextState(cur_control, wmg.X_tilde);
        //-----------------------------------------------------------



        //-----------------------------------------------------------
        // output
        ZMP_x.push_back(wmg.X_tilde[0]);
        ZMP_y.push_back(wmg.X_tilde[3]);
        CoM_x.push_back(X[0]);
        CoM_y.push_back(X[3]);
        //-----------------------------------------------------------
    


        //-----------------------------------------------------------
        // support foot and swing foot position/orientation
        double LegPos[POSITION_VECTOR_SIZE];
        double angle;
        wmg.getSwingFootPosition (
                WMG_SWING_2D_PARABOLA,
                preview_sampling_time_ms / control_sampling_time_ms,
                (preview_sampling_time_ms - next_preview_len_ms) / control_sampling_time_ms,
                LegPos,
                &angle);

        nao.initPosture (
                nao.swing_foot_posture, 
                LegPos,
                0.0,    // roll angle 
                0.0,    // pitch angle
                angle); // yaw angle

        // position of CoM
        nao.setCoM(X[0], X[3], wmg.hCoM); 


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



        //-----------------------------------------------------------
        // output
        swing_foot_x.push_back(LegPos[0]);
        swing_foot_y.push_back(LegPos[1]);
        swing_foot_z.push_back(LegPos[2]);
        /*
        fprintf(file_op, "plot3([%f %f], [%f %f], [%f %f])\n",
                LegPos[0], LegPos[0] + cos(angle)*0.005,
                LegPos[1], LegPos[1] + sin(angle)*0.005,
                LegPos[2], LegPos[2]);
        */
        //-----------------------------------------------------------
        


        next_preview_len_ms -= control_sampling_time_ms;
    }



    //-----------------------------------------------------------
    // output
    fprintf(file_op,"SFP = [\n");
    for (unsigned int i=0; i < swing_foot_x.size(); i++)
    {
        fprintf(file_op, "%f %f %f;\n", swing_foot_x[i], swing_foot_y[i], swing_foot_z[i]);
    }
    fprintf(file_op, "];\n\n plot3(SFP(:,1), SFP(:,2), SFP(:,3), 'r')\n");


    fprintf(file_op,"ZMP = [\n");
    for (unsigned int i=0; i < ZMP_x.size(); i++)
    {
        fprintf(file_op, "%f %f %f;\n", ZMP_x[i], ZMP_y[i], 0.0);
    }
    fprintf(file_op, "];\n\n plot3(ZMP(:,1), ZMP(:,2), ZMP(:,3), 'k')\n");


    fprintf(file_op,"CoM = [\n");
    for (unsigned int i=0; i < CoM_x.size(); i++)
    {
        fprintf(file_op, "%f %f %f;\n", CoM_x[i], CoM_y[i], 0.0);
    }
    fprintf(file_op, "];\n\n plot3(CoM(:,1), CoM(:,2), CoM(:,3), 'b')\n");


    fprintf(file_op,"hold off\n");
    fclose(file_op);
    //-----------------------------------------------------------

    return 0;
}

