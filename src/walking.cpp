/**
 * @file
 * @author Antonio Paolillo
 * @author Dimitar Dimitrov
 * @author Alexander Sherikov
 */

#include "mpc_walk.h"


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

    initInvPendulumModel ();
            

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
    solveMPCProblem ();


    // support foot and swing foot position/orientation
    double LegPos_des[POSITION_VECTOR_SIZE];
    double LegRot_des[ORIENTATION_MATRIX_SIZE];
    double angle;
    wmg->getSwingFootPosition (
            WMG_SWING_PARABOLA, 
            preview_sampling_time_ms / control_sampling_time_ms,
            (preview_sampling_time_ms - next_preview_len_ms) / control_sampling_time_ms,
            LegPos_des,
            &angle);
    // form the rotation matrix corresponding to a set of roll-pitch-yaw angles
    nao.rpy2R(  0.0, // roll angle 
                0.0, // pitch angle
                angle, // yaw angle
                LegRot_des); // Rotation matrix corresponding to the roll-pitch-yaw angles
    nao.initPosture (nao.swing_foot_posture, LegPos_des, LegRot_des);


    // position of CoM
    nao.CoM_position[0] = next_state[0]; // x
    nao.CoM_position[1] = next_state[3]; // y
    /// @attention hCoM is constant!
    nao.CoM_position[2] = wmg->hCoM;     // z


    // If the "current support leg" is the Right leg
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
    double foot_position[3] = {0.0, -0.05, 0.0};
    double foot_orientation[9] = {
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
    wmg->AddFootstep(0.0, -step_y/2, 0.0, 2, 2, d, FS_TYPE_DS);
    // ZMP, CoM are at [0;0]


    // all subsequent steps have normal feet size
    d[0] = 0.09;
    d[1] = 0.025;
    d[2] = 0.03;
    d[3] = 0.025;
    // 2 reference ZMP positions in single support 
    // 1 in double support
    // 1 + 2 = 3
    wmg->AddFootstep(0.0   , -step_y/2, 0.0 , 2,  3, d);
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
void mpc_walk::initInvPendulumModel ()
{
    double control_sampling_time = (double) control_sampling_time_ms / 1000;

    A[0] = A[4] = A[8] = 1;
    A[1] = A[2] = A[5] = 0;
    A[3] = A[7] = control_sampling_time;
    A[6] = control_sampling_time * control_sampling_time/2 /*- delta_hCoM = 0*/;

    B[0] = control_sampling_time * control_sampling_time * control_sampling_time / 6
        - wmg->hCoM/wmg->gravity * control_sampling_time;
    B[1] = control_sampling_time * control_sampling_time/2;
    B[2] = control_sampling_time;

    for (int i = 0; i < 6; i++)
    {
        wmg->X_tilde[i] = 0;
    }
    cur_control[0] = cur_control[1] = 0;
}



/**
 * @brief 
 */
void mpc_walk::solveMPCProblem ()
{
    // update state
    wmg->X_tilde[0] = wmg->X_tilde[0] * A[0]
                     + wmg->X_tilde[1] * A[3]
                     + wmg->X_tilde[2] * A[6]
                     + cur_control[0] * B[0];

    wmg->X_tilde[1] = wmg->X_tilde[1] * A[4]
                     + wmg->X_tilde[2] * A[7]
                     + cur_control[0] * B[1];

    wmg->X_tilde[2] = wmg->X_tilde[2] * A[8]
                     + cur_control[0] * B[2];

    wmg->X_tilde[3] = wmg->X_tilde[3] * A[0]
                     + wmg->X_tilde[4] * A[3]
                     + wmg->X_tilde[5] * A[6]
                     + cur_control[1] * B[0];

    wmg->X_tilde[4] = wmg->X_tilde[4] * A[4]
                     + wmg->X_tilde[5] * A[7]
                     + cur_control[1] * B[1];

    wmg->X_tilde[5] = wmg->X_tilde[5] * A[8]
                     + cur_control[1] * B[2];


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
    solver->form_init_fp (wmg->zref_x, wmg->zref_y, wmg->X_tilde, wmg->X);
    solver->solve();
    solver->get_next_state (next_state);
    solver->get_first_controls (cur_control);
    //------------------------------------------------------
}
