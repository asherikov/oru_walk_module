/**
 * @file
 * @author Antonio Paolillo
 * @author Dimitar Dimitrov
 * @author Alexander Sherikov
 */

#include "oru_walk.h"
#include "log_debug.h"



void oru_walk::walk()
{
    wp.readParameters();

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

    next_preview_len_ms = 0;


// solver    
    solver = new smpc::solver(
            wp.preview_window_size, // size of the preview window
            wp.mpc_alpha,
            wp.mpc_beta,
            wp.mpc_gamma,
            wp.mpc_regularization,
            wp.mpc_tolerance);
    solver->enable_fexceptions();

    com_filter = new avgFilter(wp.filter_window_length);

// models
    initWMG_NaoModel();

    ORUW_LOG_OPEN(nao.state_sensor, wp.filter_window_length);

// Connect callback to the DCM post proccess
    try
    {
        fDCMPostProcessConnection =
            getParentBroker()->getProxy("DCM")->getModule()->atPostProcess
            (boost::bind(&oru_walk::callbackEveryCycle_walk, this));
    }
    catch (const ALError &e)
    {
        ORUW_HALT("Error when connecting to DCM postProccess: " + string(e.what()));
    }
}


void oru_walk::halt(const string &message, const char* function)
{
    stopWalking(message);
    setStiffness(0.0);
}


void oru_walk::stopWalking(const string& message)
{
    ORUW_LOG_MESSAGE("%s", message.c_str());
    qiLogInfo ("module.oru_walk") << message;
    fDCMPostProcessConnection.disconnect();
    ORUW_LOG_CLOSE;
}

void oru_walk::stopWalkingRemote()
{
    qiLogInfo ("module.oru_walk", "Stopped by user's request.\n");
    fDCMPostProcessConnection.disconnect();
    ORUW_LOG_CLOSE;
}


/**
 * @brief 
 * @attention REAL-TIME!
 */
void oru_walk::callbackEveryCycle_walk()
{
    ORUW_TIMER(wp.loop_time_limit_ms);
    readSensors (nao.state_sensor);

    ORUW_LOG_JOINTS(nao.state_sensor, nao.state_model);
    ORUW_LOG_COM(wmg, nao.state_sensor);
    ORUW_LOG_FEET(nao);
    ORUW_LOG_JOINT_VELOCITIES(nao.state_sensor, (double) wp.control_sampling_time_ms/1000);

    feedbackError ();
    double joint_error_feedback[LOWER_JOINTS_NUM];
    for (int i = 0; i < LOWER_JOINTS_NUM; i++)
    {
        joint_error_feedback[i] = wp.joint_feedback_gain * (nao.state_model.q[i] - nao.state_sensor.q[i]);
    }

    if (! solveMPCProblem ())
    {
        return;
    }


    // support foot and swing foot position/orientation
    double left_foot_pos[POSITION_VECTOR_SIZE + 1];
    double right_foot_pos[POSITION_VECTOR_SIZE + 1];
    wmg->getFeetPositions (
            wp.preview_sampling_time_ms/wp.control_sampling_time_ms,
            (wp.preview_sampling_time_ms - next_preview_len_ms)/wp.control_sampling_time_ms,
            left_foot_pos,
            right_foot_pos);
    nao.setFeetPostures (left_foot_pos, right_foot_pos);


    // position of CoM
    /// @attention hCoM is constant!
    nao.setCoM(wmg->init_state.x(), wmg->init_state.y(), wmg->hCoM);


    if (nao.igm () < 0)
    {
        ORUW_HALT("IK does not converge.\n");
    }

    int failed_joint = nao.checkJointBounds();
    if (failed_joint >= 0)
    {
        ORUW_LOG_MESSAGE("Failed joint: %d\n", failed_joint);
        ORUW_HALT("Joint bounds are violated.\n");
    }


    // Set commands
    try
    {
        walkCommands[4][0] = dcmProxy->getTime(wp.control_sampling_time_ms);
        for (int i = 0; i < LOWER_JOINTS_NUM; i++)
        {
            walkCommands[5][i][0] = nao.state_model.q[i] + joint_error_feedback[i];
        }
        dcmProxy->setAlias(walkCommands);
    }
    catch (const AL::ALError &e)
    {
        ORUW_HALT("Cannot set joint angles: " + string(e.what()));
    }

    next_preview_len_ms -= wp.control_sampling_time_ms;
    ORUW_TIMER_CHECK;
}



/**
 * @brief Correct state and the model based on the sensor data.
 */
void oru_walk::feedbackError ()
{
    double CoM_pos[POSITION_VECTOR_SIZE];
    nao.state_sensor.getCoM (CoM_pos);

    //com_filter->addValue(CoM_pos[0], CoM_pos[1], state_sensor.x(), state_sensor.y());

    smpc::state_orig state_error;
    state_error.set (
            wmg->init_state.x() - CoM_pos[0],
            wmg->init_state.y() - CoM_pos[1]);

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

    wmg->init_state.x() -= wp.feedback_gain * state_error.x();
    wmg->init_state.y() -= wp.feedback_gain * state_error.y();
}




/**
 * @brief Update joint angles in the NAO model.
 */
void oru_walk::readSensors(modelState& nao_state)
{
    accessSensorValues->GetValues (sensorValues);
    for (int i = 0; i < JOINTS_NUM; i++)
    {
        nao_state.q[i] = sensorValues[i];
    }
}



/**
 * @brief 
 * @attention Hardcoded parameters.
 */
void oru_walk::initWMG_NaoModel()
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
    wmg->AddFootstep(0.0   , -step_y/2, 0.0 , wp.ss_number,  wp.ss_number + wp.ds_number, d);
    wmg->AddFootstep(step_x,  step_y, 0.0);

    for (int i = 0; i < wp.step_pairs_number; i++)
    {
        wmg->AddFootstep(step_x, -step_y, 0.0);
        wmg->AddFootstep(step_x,  step_y, 0.0);
    }

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
    readSensors(nao.state_sensor);

    // support foot position and orientation
    nao.init (
            IGM_SUPPORT_LEFT,
            0.0, 0.05, 0.0, // position
            0.0, 0.0, 0.0);  // orientation
    
    wmg->init_param (     
            (double) wp.preview_sampling_time_ms / 1000, // sampling time in seconds
            nao.CoM_position[2],                      // height of the center of mass
            wp.step_height);                // step height (for interpolation of feet movements)

    double pos_error[POSITION_VECTOR_SIZE];
    nao.state_sensor.getSwingFootPosition (pos_error);
    pos_error[0] =  0.0  - pos_error[0];
    pos_error[1] = -0.05 - pos_error[1];
    pos_error[2] =  0.0;//  - pos_error[2];
    wmg->correctNextSSPosition (pos_error);

    wmg->initABMatrices ((double) wp.control_sampling_time_ms / 1000);
    wmg->init_state.set (nao.CoM_position[0], nao.CoM_position[1]);
}



/**
 * @brief 
 */
bool oru_walk::solveMPCProblem ()
{
    ORUW_TIMER(wp.loop_time_limit_ms);

    if (next_preview_len_ms == 0)
    {
        if (wmg->isSupportSwitchNeeded())
        {
            double pos_error[POSITION_VECTOR_SIZE];
            nao.switchSupportFoot(pos_error);
            wmg->correctNextSSPosition(pos_error);
        }

        WMGret wmg_retval = wmg->formPreviewWindow();


        if (wmg_retval == WMG_HALT)
        {
            stopWalking("Not enough steps to form preview window. Stopping.");
            return (false);
        }


        next_preview_len_ms = wp.preview_sampling_time_ms;
    }

    wmg->T[0] = (double) next_preview_len_ms / 1000; // get seconds
    //------------------------------------------------------
    solver->set_parameters (wmg->T, wmg->h, wmg->h[0], wmg->angle, wmg->zref_x, wmg->zref_y, wmg->lb, wmg->ub);
    solver->form_init_fp (wmg->fp_x, wmg->fp_y, wmg->init_state, wmg->X);
    int num_iq_constr = solver->solve();
    ORUW_LOG_MESSAGE("Num of active constraints: %d\n", num_iq_constr);
    //------------------------------------------------------
    // update state
    wmg->next_control.get_first_controls (*solver);
    wmg->calculateNextState(wmg->next_control, wmg->init_state);
    //------------------------------------------------------
    
    ORUW_TIMER_CHECK;
    return (true);
}
