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
    int control_sampling_time_ms = 20;
    int preview_sampling_time_ms = 40;
    int next_preview_len_ms = 0;
    //-----------------------------------------------------------



    //-----------------------------------------------------------
    // initialize classes
    nao_igm nao;
    double ref_angles[LOWER_JOINTS_NUM];
    initNaoModel (&nao, ref_angles);
    nao.init (
            IGM_SUPPORT_RIGHT,
            0.0, -0.05, 0.0,
            0.0, 0.0, 0.0);
    nao.getCoM(nao.state_sensor, nao.CoM_position);
    init_08 test_04("test_04", preview_sampling_time_ms, nao.CoM_position[2], false);


    smpc::solver solver(
            test_04.wmg->N, // size of the preview window
            1.0,  // Alpha
            8000.0,  // Beta
            1.0,    // Gamma
            0.01,   // regularization
            1e-7);  // tolerance
    //-----------------------------------------------------------



    //-----------------------------------------------------------
    test_04.par->init_state.set (nao.CoM_position[0], nao.CoM_position[1]);
    test_04.X_tilde.set (nao.CoM_position[0], nao.CoM_position[1]);
    //-----------------------------------------------------------



    //-----------------------------------------------------------
    // output
    FILE *file_op = fopen(test_04.fs_out_filename.c_str(), "a");
    fprintf(file_op,"hold on\n");

    vector<double> ZMP_x;
    vector<double> ZMP_y;
    vector<double> ZMPref_x;
    vector<double> ZMPref_y;
    vector<double> CoM_x;
    vector<double> CoM_y;

    vector<double> left_foot_x;
    vector<double> left_foot_y;
    vector<double> left_foot_z;
    vector<double> right_foot_x;
    vector<double> right_foot_y;
    vector<double> right_foot_z;
    //-----------------------------------------------------------




    test_04.wmg->T_ms[0] = control_sampling_time_ms;
    test_04.wmg->T_ms[1] = control_sampling_time_ms;
    for(int i=0 ;; i++)
    {
        nao.state_sensor = nao.state_model;



        if (test_04.wmg->formPreviewWindow(*test_04.par) == WMG_HALT)
        {
            cout << "EXIT (halt = 1)" << endl;
            break;
        }
        for (unsigned int j = 0; j < test_04.wmg->N; j++)
        {
            ZMPref_x.push_back(test_04.par->zref_x[j]);
            ZMPref_y.push_back(test_04.par->zref_y[j]);
        }
        cout << test_04.wmg->isSupportSwitchNeeded() << endl;
        if (test_04.wmg->isSupportSwitchNeeded())
        {
            nao.switchSupportFoot();
        }

       
        
        //------------------------------------------------------
        solver.set_parameters (test_04.par->T, test_04.par->h, test_04.par->h[0], test_04.par->angle, test_04.par->zref_x, test_04.par->zref_y, test_04.par->lb, test_04.par->ub);
        solver.form_init_fp (test_04.par->fp_x, test_04.par->fp_y, test_04.par->init_state, test_04.par->X);
        solver.solve();
        //-----------------------------------------------------------
        // update state
        test_04.par->init_state.get_next_state (solver);
        //-----------------------------------------------------------



        //-----------------------------------------------------------
        // output
        if (next_preview_len_ms == 0)
        {
            next_preview_len_ms = preview_sampling_time_ms;

            ZMP_x.push_back(test_04.X_tilde.x());
            ZMP_y.push_back(test_04.X_tilde.y());
            test_04.X_tilde.get_next_state (solver);
        }
        CoM_x.push_back(test_04.par->init_state.x());
        CoM_y.push_back(test_04.par->init_state.y());

        next_preview_len_ms -= control_sampling_time_ms;
        //-----------------------------------------------------------
    


        //-----------------------------------------------------------
        // support foot and swing foot position/orientation
        test_04.wmg->getFeetPositions (
                control_sampling_time_ms,
                nao.left_foot_posture->data(),
                nao.right_foot_posture->data());

        // position of CoM
        nao.setCoM(test_04.par->init_state.x(), test_04.par->init_state.y(), test_04.par->hCoM); 


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
        // output
        left_foot_x.push_back(nao.left_foot_posture->data()[12]);
        left_foot_y.push_back(nao.left_foot_posture->data()[13]);
        left_foot_z.push_back(nao.left_foot_posture->data()[14]);
        right_foot_x.push_back(nao.right_foot_posture->data()[12]);
        right_foot_y.push_back(nao.right_foot_posture->data()[13]);
        right_foot_z.push_back(nao.right_foot_posture->data()[14]);
        //-----------------------------------------------------------
       

        //-----------------------------------------------------------
        nao_igm nao_next = nao;

        test_04.wmg->getFeetPositions (
                2*control_sampling_time_ms,
                nao_next.left_foot_posture->data(),
                nao_next.right_foot_posture->data());

        // position of CoM
        smpc::state_orig next_CoM;
        next_CoM.get_state(solver, 1);
        nao_next.setCoM(next_CoM.x(), next_CoM.y(), test_04.par->hCoM); 


        if (nao_next.igm(ref_angles, 1.2, 0.0015, 20) < 0)
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
    }



    //-----------------------------------------------------------
    // output
    printVectors (file_op, left_foot_x, left_foot_y, left_foot_z, "LFP", "r");
    printVectors (file_op, right_foot_x, right_foot_y, right_foot_z, "RFP", "r");
    printVectors (file_op, ZMP_x, ZMP_y, "ZMP", "k");
    printVectors (file_op, ZMPref_x, ZMPref_y, "ZMPref", "x");
    printVectors (file_op, CoM_x, CoM_y, "CoM", "b");
    fprintf(file_op,"hold off\n");
    fclose(file_op);
    //-----------------------------------------------------------

    return 0;
}

