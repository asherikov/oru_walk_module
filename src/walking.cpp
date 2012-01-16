/**
 * @file
 * @author Antonio Paolillo
 * @author Dimitar Dimitrov
 * @author Alexander Sherikov
 */

#include "oru_walk.h"
#include "log_debug.h"

/**
 * @brief Changes specified parameters.
 *
 * @param[in] feedback_gain_ feedback gain
 * @param[in] feedback_threshold_ feedback threshold
 * @param[in] mpc_alpha_ alpha gain for the QP objective
 * @param[in] mpc_beta_ beta gain for the QP objective
 * @param[in] mpc_gamma_ gamma gain for the QP objective
 * @param[in] step_height_ height of a step
 */
void oru_walk::setWalkParameters (
        const float &feedback_gain_,
        const float &feedback_threshold_,
        const float &mpc_alpha_,
        const float &mpc_beta_,
        const float &mpc_gamma_,
        const float &step_height_)
{
    wp.set (
        feedback_gain_,
        feedback_threshold_,
        mpc_alpha_,
        mpc_beta_,
        mpc_gamma_,
        step_height_);
}




void oru_walk::walk()
{
    next_preview_len_ms = 0;


// solver    
    solver = new smpc::solver(
            wp.preview_window_size, // size of the preview window
            wp.mpc_alpha,
            wp.mpc_beta,
            wp.mpc_gamma,
            wp.mpc_regularization,
            wp.mpc_tolerance);

    com_filter = new avgFilter(wp.filter_window_length);

// models
    initSteps_NaoModel();
    wmg->init_param (     
            (double) wp.preview_sampling_time_ms / 1000, // sampling time in seconds
            nao.CoM_position[2],                      // height of the center of mass
            wp.step_height);                // step height (for interpolation of feet movements)

    wmg->initABMatrices ((double) wp.control_sampling_time_ms / 1000);
    wmg->init_state.set (nao.CoM_position[0], nao.CoM_position[1]);
    old_state = wmg->init_state;


    ORUW_LOG_OPEN(wp.filter_window_length);

// Connect callback to the DCM post proccess
    try
    {
        fDCMPostProcessConnection =
            getParentBroker()->getProxy("DCM")->getModule()->atPostProcess
            (boost::bind(&oru_walk::callbackEveryCycle_walk, this));
    }
    catch (const AL::ALError &e)
    {
        throw ALERROR(getName(), __FUNCTION__, 
                "Error when connecting to DCM postProccess: " + e.toString());
    }
}


void oru_walk::stopWalking()
{
    fDCMPostProcessConnection.disconnect();

    ORUW_LOG_CLOSE;

    if (solver != NULL)
    {
        delete solver;
        solver = NULL;
    }

    if (wmg != NULL)
    {
        delete wmg;
        wmg = NULL;
    }

    if (com_filter != NULL)
    {
        delete com_filter;
        com_filter = NULL;
    }
}



/**
 * @brief 
 * @attention REAL-TIME!
 */
void oru_walk::callbackEveryCycle_walk()
{
    ORUW_TIMER(__FUNCTION__, 9000);
    ORUW_LOG_JOINTS(accessSensorValues, accessActuatorValues);
    ORUW_LOG_COM(wmg, nao, accessSensorValues);
    ORUW_LOG_SWING_FOOT(nao, accessSensorValues);

    correctStateAndModel ();
    //updateModelJoints ();

    if (! solveMPCProblem ())
    {
        return;
    }


    // support foot and swing foot position/orientation
    double swing_foot_pos[POSITION_VECTOR_SIZE];
    double angle;
    wmg->getSwingFootPosition (
            WMG_SWING_2D_PARABOLA, 
            wp.preview_sampling_time_ms/wp.control_sampling_time_ms,
            (wp.preview_sampling_time_ms - next_preview_len_ms)/wp.control_sampling_time_ms,
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
    nao.setCoM(wmg->init_state.x(), wmg->init_state.y(), wmg->hCoM);


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


    for (int i = 0; i < LOWER_JOINTS_NUM; i++)
    {
        walkCommands[5][i][0] = nao.q[i];
    }


    // Get time
    try
    {
        walkCommands[4][0] = dcmProxy->getTime(wp.control_sampling_time_ms);
        dcmProxy->setAlias(walkCommands);
    }
    catch (const AL::ALError &e)
    {
        stopWalking();
        throw ALERROR(getName(), __FUNCTION__, "Cannot set joint angles: " + e.toString());
    }

    next_preview_len_ms -= wp.control_sampling_time_ms;
}



/**
 * @brief Correct state and the model based on the sensor data.
 */
void oru_walk::correctStateAndModel ()
{
    updateModelJoints ();

    double CoM_pos[POSITION_VECTOR_SIZE];
    nao.getUpdatedCoM (CoM_pos);

    smpc::state_orig state_sensor;
    com_filter->addValue(CoM_pos[0], CoM_pos[1], state_sensor.x(), state_sensor.y());

    smpc::state_orig state_error;
    state_error.set (
            wmg->init_state.x()  - state_sensor.x(),
            wmg->init_state.y()  - state_sensor.y());

    if (state_error.x() > wp.feedback_threshold)
    {
        state_error.x() -= wp.feedback_threshold;
    }
    else if (state_error.x() < -wp.feedback_threshold)
    {
        state_error.x() += wp.feedback_threshold;
    }
    else
    {
        state_error.x() = 0.0;
    }

    if (state_error.y() > wp.feedback_threshold)
    {
        state_error.y() -= wp.feedback_threshold;
    }
    else if (state_error.y() < -wp.feedback_threshold)
    {
        state_error.y() += wp.feedback_threshold;
    }
    else
    {
        state_error.y() = 0.0;
    }

    wmg->init_state.x()  -= wp.feedback_gain * state_error.x();
    wmg->init_state.y()  -= wp.feedback_gain * state_error.y();

    old_state = wmg->init_state;
}




/**
 * @brief Update joint angles in the NAO model.
 */
void oru_walk::updateModelJoints()
{
    accessSensorValues->GetValues (sensorValues);
    for (int i = 0; i < JOINTS_NUM; i++)
    {
        nao.q[i] = sensorValues[i];
    }
}



/**
 * @brief 
 * @attention Hardcoded parameters.
 */
void oru_walk::initSteps_NaoModel()
{
    wmg = new WMG();
    wmg->init(wp.preview_window_size);     // size of the preview window

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
    wmg->AddFootstep(0.0, -step_y/2, 0.0, 10, 10, d, FS_TYPE_DS);
    // ZMP, CoM are at [0;0]


    // all subsequent steps have normal feet size
    d[0] = 0.09;
    d[1] = 0.025;
    d[2] = 0.03;
    d[3] = 0.025;
    // 2 reference ZMP positions in single support 
    // 1 in double support
    // 1 + 2 = 3
    wmg->AddFootstep(0.0   , -step_y/2, 0.0 , 20,  25, d);
    wmg->AddFootstep(step_x,  step_y, 0.0);
    wmg->AddFootstep(step_x, -step_y, 0.0);
    wmg->AddFootstep(step_x,  step_y, 0.0);
    wmg->AddFootstep(step_x, -step_y, 0.0);
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
    wmg->AddFootstep(0.0   , -step_y/2, 0.0, 140, 140, d, FS_TYPE_DS);
    d[0] = 0.09;
    d[1] = 0.025;
    d[2] = 0.03;
    d[3] = 0.025;
    wmg->AddFootstep(0.0   , -step_y/2, 0.0 , 0,  0, d, FS_TYPE_SS_R);


// Nao
    updateModelJoints();

    // support foot position and orientation
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
bool oru_walk::solveMPCProblem ()
{
    if (next_preview_len_ms == 0)
    {
        bool switch_foot = false;
        WMGret wmg_retval = wmg->FormPreviewWindow(&switch_foot);

        if (wmg_retval == WMG_HALT)
        {
            stopWalking();
            return (false);
        }

        if (switch_foot)
        {
            nao.switchSupportFoot();
        }

        next_preview_len_ms = wp.preview_sampling_time_ms;
    }

    wmg->T[0] = (double) next_preview_len_ms / 1000; // get seconds
    //------------------------------------------------------
    solver->set_parameters (wmg->T, wmg->h, wmg->h[0], wmg->angle, wmg->zref_x, wmg->zref_y, wmg->lb, wmg->ub);
    solver->form_init_fp (wmg->fp_x, wmg->fp_y, wmg->init_state, wmg->X);
    solver->solve();
    //------------------------------------------------------
    // update state
    wmg->next_control.get_first_controls (*solver);
    wmg->calculateNextState(wmg->next_control, wmg->init_state);
    //------------------------------------------------------
    
    return (true);
}
