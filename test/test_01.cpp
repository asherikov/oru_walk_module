/** 
 * @file
 * @author Alexander Sherikov
 */
#include <iostream>
#include <fstream>
#include <cstdio>


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
    int preview_sampling_time_ms = 100;
    int next_preview_len_ms = 0;
    //-----------------------------------------------------------



    //-----------------------------------------------------------
    // initialize classes
    init_04 tdata ("test_01", preview_sampling_time_ms);
    IPM ipm ((double) control_sampling_time_ms / 1000);


    smpc::solver solver(
            tdata.wmg->N, // size of the preview window
            300.0,  // Alpha
            800.0,  // Beta
            1.0,    // Gamma
            0.01,   // regularization
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



    for(int i=0 ;; i++)
    {
        tdata.nao.state_sensor = tdata.nao.state_model;
        if (next_preview_len_ms == 0)
        {

            if ( tdata.wmg->formPreviewWindow(*tdata.par) == WMG_HALT)
            {
                cout << "EXIT (halt = 1)" << endl;
                break;
            }
            if (tdata.wmg->isSupportSwitchNeeded())
            {
                tdata.nao.switchSupportFoot();
            }
            for (unsigned int j = 0; j < tdata.wmg->N; j++)
            {
                log.addZMPrefPoint(tdata.par->zref_x[j], tdata.par->zref_y[j]);
            }

            next_preview_len_ms = preview_sampling_time_ms;
        }   

       
        
        //------------------------------------------------------
        tdata.par->T[0] = (double) next_preview_len_ms / 1000; // get seconds
        solver.set_parameters (tdata.par->T, tdata.par->h, tdata.par->h[0], tdata.par->angle, tdata.par->zref_x, tdata.par->zref_y, tdata.par->lb, tdata.par->ub);
        solver.form_init_fp (tdata.par->fp_x, tdata.par->fp_y, tdata.par->init_state, tdata.par->X);
        solver.solve();
        //-----------------------------------------------------------
        // update state
        ipm.control_vector.get_first_controls (solver);
        ipm.calculateNextState(ipm.control_vector, tdata.par->init_state);
        //-----------------------------------------------------------



        //-----------------------------------------------------------
        // output
        if (next_preview_len_ms == preview_sampling_time_ms)
        {
            log.addZMPpoint (tdata.X_tilde.x(), tdata.X_tilde.y());
            tdata.X_tilde.get_next_state (solver);
        }
        log.addCoMpoint (tdata.par->init_state.x(), tdata.par->init_state.y());
        //-----------------------------------------------------------
    


        //-----------------------------------------------------------
        // support foot and swing foot position/orientation
        tdata.wmg->getFeetPositions (
                preview_sampling_time_ms - next_preview_len_ms + control_sampling_time_ms,
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


        log.addFeetPositions (tdata.nao);

        next_preview_len_ms -= control_sampling_time_ms;
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

