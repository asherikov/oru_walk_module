/**
 * @file
 * @author Antonio Paolillo
 * @author Dimitar Dimitrov
 * @author Alexander Sherikov
 */

#include "mpc_walk.h"


/**
 * Measure execution time of the callback function.
 * By default naoqi prints logs to stdout, when executed on a PC,
 * the location of the log file on the robot is /var/log/naoqi.log
 */
#define ORU_MEASURE_EXEC_TIME
#ifdef ORU_MEASURE_EXEC_TIME
#include <qi/os.hpp>
#include <qi/log.hpp>
#endif


void mpc_walk::walk()
{
    /// @attention Hardcoded parameters.
    control_sampling_time_ms = 10;
    preview_sampling_time_ms = 100;
    next_preview_len_ms = 0;
    preview_window_size = 15;


// WMG
    initWMG ();


// solver    
    if (solver != NULL)
    {
        delete solver;
    }
    /// @attention Hardcoded parameters.
    solver = new smpc_solver(
            wmg->N, // size of the preview window
            300.0,  // Alpha
            800.0,  // Beta
            1.0,    // Gamma
            0.01,   // regularization
            1e-7);  // tolerance


// models
    initNaoModel ();
    wmg->init_param (     
            (double) preview_sampling_time_ms / 1000, // sampling time in seconds
            nao.CoM_position[2],                      // height of the center of mass
            0.0135);                // step hight (for interpolation of feet movements)
    /// @ref AldNaoPaper "0.015 in the paper"

    wmg->initABMatrices ((double) control_sampling_time_ms / 1000);
    wmg->initState (nao.CoM_position[0], nao.CoM_position[1], wmg->X_tilde);
    cur_control[0] = cur_control[1] = 0;


// Connect callback to the DCM post proccess
    try
    {
        fDCMPostProcessConnection =
            getParentBroker()->getProxy("DCM")->getModule()->atPostProcess
            (boost::bind(&mpc_walk::callbackEveryCycle_walk, this));

    }
    catch (const AL::ALError &e)
    {
        throw ALERROR(getName(), __FUNCTION__, 
                "Error when connecting to DCM postProccess: " + e.toString());
    }
}


void mpc_walk::stopWalking()
{
    fDCMPostProcessConnection.disconnect();
}


/**
 * @brief 
 * @attention REAL-TIME!
 * @todo set commands in advance
 */
void mpc_walk::callbackEveryCycle_walk()
{
#ifdef ORU_MEASURE_EXEC_TIME
    qi::os::timeval start_time;
    qi::os::gettimeofday (&start_time);
#endif

    solveMPCProblem ();


    // support foot and swing foot position/orientation
    double swing_foot_pos[POSITION_VECTOR_SIZE];
    double angle;
    wmg->getSwingFootPosition (
            WMG_SWING_PARABOLA, 
            1,
            1,
            swing_foot_pos,
            &angle);
    nao.initPosture (
            nao.swing_foot_posture, 
            swing_foot_pos, 
            0.0,    // roll angle 
            0.0,    // pitch angle
            angle); // yaw angle


    // position of CoM
    /// @attention hCoM is constant!
    nao.setCoM(next_state[0], next_state[3], wmg->hCoM);


    if (nao.igm_3(nao.swing_foot_posture, nao.CoM_position, nao.torso_orientation) < 0)
    {
        stopWalking();
        throw ALERROR(getName(), __FUNCTION__, "IK does not converge.");
    }

    if (nao.checkJointBounds() >= 0)
    {
        stopWalking();
        throw ALERROR(getName(), __FUNCTION__, "Joint bounds are violated.");
    }


    for (int i = 0; i < JOINTS_NUM; i++)
    {
        walkCommands[5][i][0] = nao.q[i];
    }


    // Get time
    try
    {
        walkCommands[4][0] = dcmProxy->getTime(next_preview_len_ms);
    }
    catch (const AL::ALError &e)
    {
        throw ALERROR(getName(), __FUNCTION__, "Error on DCM getTime : " + e.toString());
    }


    // Set commands
    try
    {
        dcmProxy->setAlias(walkCommands);
    }
    catch (const AL::ALError &e)
    {
        throw ALERROR(getName(), __FUNCTION__, "Error with DCM setAlias : " + e.toString());
    }


    next_preview_len_ms -= control_sampling_time_ms;

#ifdef ORU_MEASURE_EXEC_TIME
    qi::os::timeval end_time;
    qi::os::gettimeofday (&end_time);
//    qiLogInfo ("module.oru_walk", "Exec time: %d x %d -- %d x %d\n", 
//        end_time.tv_sec, start_time.tv_sec, end_time.tv_usec, start_time.tv_usec);
    qiLogInfo ("module.oru_walk", "Exec time (sec): %f\n", 
        (double) end_time.tv_sec - start_time.tv_sec + 0.000001* (end_time.tv_usec - start_time.tv_usec));
#endif
}



/**
 * @brief 
 */
void mpc_walk::initNaoModel ()
{
    // read joint anglesand initialize model
    fMemoryFastAccess->GetValues(sensorValues);
    for (int i = 0; i < JOINTS_NUM; i++)
    {
        nao.q[i] = sensorValues[i];
    }


    // support foot position and orientation
    /// @attention Hardcoded parameters.
    double foot_position[POSITION_VECTOR_SIZE] = {0.0, -0.05, 0.0};
    double foot_orientation[ORIENTATION_MATRIX_SIZE] = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0};
    nao.init (
            IGM_SUPPORT_RIGHT,
            foot_position, 
            foot_orientation);
}



/**
 * @brief 
 */
void mpc_walk::initWMG ()
{
    if (wmg != NULL)
    {
        delete wmg;
    }
    wmg = new WMG();
    wmg->init(preview_window_size);     // size of the preview window

    double d[4];

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
    d[3] = 0.025;
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
    d[1] = 0.025;
    d[2] = 0.03;
    d[3] = 0.075;
    wmg->AddFootstep(0.0   , -step_y/2, 0.0, 30, 30, d, FS_TYPE_DS);
    d[0] = 0.09;
    d[1] = 0.025;
    d[2] = 0.03;
    d[3] = 0.025;
    wmg->AddFootstep(0.0   , -step_y/2, 0.0 , 0,  0, d, FS_TYPE_SS_R);
}



/**
 * @brief 
 */
void mpc_walk::solveMPCProblem ()
{
    if (next_preview_len_ms == 0)
    {
        WMGret wmg_retval = wmg->FormPreviewWindow();

        if (wmg_retval == WMG_HALT)
        {
            stopWalking();
            return;
        }

        if (wmg_retval == WMG_SWITCH_REFERENCE_FOOT)
        {
            nao.switchSupportFoot();
        }

        next_preview_len_ms = preview_sampling_time_ms;
    }

    wmg->T[0] = (double) next_preview_len_ms / 1000; // get seconds
    //------------------------------------------------------
    solver->set_parameters (wmg->T, wmg->h, wmg->angle, wmg->zref_x, wmg->zref_y, wmg->lb, wmg->ub);
    solver->form_init_fp (wmg->fp_x, wmg->fp_y, wmg->X_tilde, wmg->X);
    solver->solve();
    solver->get_next_state (next_state);
    solver->get_first_controls (cur_control);
    //------------------------------------------------------

    // update state
    wmg->calculateNextState(cur_control, wmg->X_tilde);
}
