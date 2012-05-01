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
    init_08 tdata("test_04", preview_sampling_time_ms, false);


    smpc::solver_as solver(
            tdata.wmg->N, // size of the preview window
            8000.0,  // Beta
            1.0,    // Alpha
            0.02,   // regularization
            1.0,    // Gamma
            1e-7);  // tolerance
    //-----------------------------------------------------------



    //-----------------------------------------------------------
    tdata.nao.getCoM(tdata.nao.state_sensor, tdata.nao.CoM_position);
    tdata.par->init_state.set (tdata.nao.CoM_position[0], tdata.nao.CoM_position[1]);
    tdata.X_tilde.set (tdata.nao.CoM_position[0], tdata.nao.CoM_position[1]);
    //-----------------------------------------------------------



    //-----------------------------------------------------------
    // output
    test_log log(tdata.fs_out_filename.c_str());
    //-----------------------------------------------------------




    tdata.wmg->T_ms[0] = control_sampling_time_ms;
    tdata.wmg->T_ms[1] = control_sampling_time_ms;
    for(int i=0 ;; i++)
    {
        tdata.nao.state_sensor = tdata.nao.state_model;



        if (tdata.wmg->formPreviewWindow(*tdata.par) == WMG_HALT)
        {
            cout << "EXIT (halt = 1)" << endl;
            break;
        }
        for (unsigned int j = 0; j < tdata.wmg->N; j++)
        {
            log.addZMPrefPoint(tdata.par->zref_x[j], tdata.par->zref_y[j]);
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
        // output
        if (next_preview_len_ms == 0)
        {
            next_preview_len_ms = preview_sampling_time_ms;

            log.addZMPpoint (tdata.X_tilde.x(), tdata.X_tilde.y());
            solver.get_next_state(tdata.X_tilde);
        }
        log.addCoMpoint (tdata.par->init_state.x(), tdata.par->init_state.y());

        next_preview_len_ms -= control_sampling_time_ms;
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
        // output
        log.addFeetPositions (tdata.nao);
        //-----------------------------------------------------------
       

        //-----------------------------------------------------------
        tdata.wmg->getFeetPositions (
                2*control_sampling_time_ms,
                tdata.nao.left_foot_posture.data(),
                tdata.nao.right_foot_posture.data());

        // position of CoM
        smpc::state_orig next_CoM;
        solver.get_state(next_CoM, 1);
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
    }



    //-----------------------------------------------------------
    // output
    log.flushLeftFoot ();
    log.flushRightFoot ();
    log.flushZMP ();
    log.flushZMPref ();
    log.flushCoM ();
    //-----------------------------------------------------------

    return 0;
}

