/**
 * @file
 * @author Antonio Paolillo
 * @author Dimitar Dimitrov
 * @author Alexander Sherikov
 */

#include "oru_walk.h"
#include "log_debug.h"



/**
 * @brief Construct and initialize all necessary classes and register 
 * callback function.
 */
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
            getParentBroker()->getProxy("ALMotion")->getModule()->atPreProcess
            (boost::bind(&oru_walk::callbackEveryCycle_walk, this));
    }
    catch (const ALError &e)
    {
        halt("Error when connecting to DCM postProccess: " + string(e.what()), __FUNCTION__);
    }
}



/**
 * @brief Log a message, remove stiffness and die.
 *
 * @param[in] message a message
 * @param[in] function name of the calling function.
 */
void oru_walk::halt(const string &message, const char* function)
{
    stopWalking(message);
    setStiffness(0.0);
    throw ALERROR(getName(), function, message);
}



/**
 * @brief Unregister callback and log a message.
 *
 * @param[in] message a message
 */
void oru_walk::stopWalking(const string& message)
{
    ORUW_LOG_STEPS;
    ORUW_LOG_MESSAGE("%s", message.c_str());
    qiLogInfo ("module.oru_walk") << message;
    fDCMPostProcessConnection.disconnect();
    ORUW_LOG_CLOSE;
}



/**
 * @brief An interface function that is called remotely to stop
 * the execution.
 */
void oru_walk::stopWalkingRemote()
{
    qiLogInfo ("module.oru_walk", "Stopped by user's request.\n");
    fDCMPostProcessConnection.disconnect();
    ORUW_LOG_CLOSE;
}



/**
 * @brief A periodically called function, that determines and sends 
 * appropriate commands to the joints.
 * @attention REAL-TIME!
 */
void oru_walk::callbackEveryCycle_walk()
{
    ORUW_TIMER(wp.loop_time_limit_ms);


    // execution of the commands must finish when the next call to the
    // callback is made
    walkCommands[4][0] = dcmProxy->getTime(wp.control_sampling_time_ms);
    walkCommands[4][1] = dcmProxy->getTime(2*wp.preview_sampling_time_ms - next_preview_len_ms);

    readSensors (nao.state_sensor);


    ORUW_LOG_JOINTS(nao.state_sensor, nao.state_model);
    ORUW_LOG_COM(wmg, nao.state_sensor);
    ORUW_LOG_FEET(nao);
    ORUW_LOG_JOINT_VELOCITIES(nao.state_sensor, wp.control_sampling_time_sec);


    double left_foot_pos[POSITION_VECTOR_SIZE + 1];
    double right_foot_pos[POSITION_VECTOR_SIZE + 1];
    int failed_joint;



    // position of CoM
    feedbackError ();
    if (! solveMPCProblem ())
    {
        return;
    }
    /// @attention hCoM is constant!
    nao.setCoM(wmg->init_state.x(), wmg->init_state.y(), wmg->hCoM);

    // support foot and swing foot position/orientation
    wmg->getFeetPositions (
            0,
            wp.preview_sampling_time_ms/wp.control_sampling_time_ms,
            (wp.preview_sampling_time_ms - next_preview_len_ms)/wp.control_sampling_time_ms,
            left_foot_pos,
            right_foot_pos);
    nao.setFeetPostures (left_foot_pos, right_foot_pos);

    if (nao.igm (nao.state_model) < 0)
    {
        halt("IK does not converge.\n", __FUNCTION__);
    }

    failed_joint = nao.state_model.checkJointBounds();
    if (failed_joint >= 0)
    {
        ORUW_LOG_MESSAGE("Failed joint: %d\n", failed_joint);
        halt("Joint bounds are violated.\n", __FUNCTION__);
    }



    /// @attention hCoM is constant!
    smpc::state_orig CoM;
    CoM.get_state(*solver, 1);
    nao.setCoM(CoM.x(), CoM.y(), wmg->hCoM);


    // support foot and swing foot position/orientation
    wmg->getFeetPositions (1, 1, 0, left_foot_pos, right_foot_pos);
    nao.setFeetPostures (left_foot_pos, right_foot_pos);


    // inverse kinematics    
    modelState state_copy = nao.state_model;
    if (nao.igm (state_copy) < 0)
    {
        halt("IK does not converge.\n", __FUNCTION__);
    }
    failed_joint = state_copy.checkJointBounds();
    if (failed_joint >= 0)
    {
        ORUW_LOG_MESSAGE("Failed joint: %d\n", failed_joint);
        halt("Joint bounds are violated.\n", __FUNCTION__);
    }


    // Set commands
    try
    {
        for (int i = 0; i < LOWER_JOINTS_NUM; i++)
        {
            walkCommands[5][i][0] = nao.state_model.q[i];
            walkCommands[5][i][1] = state_copy.q[i];
        }
        dcmProxy->setAlias(walkCommands);
    }
    catch (const AL::ALError &e)
    {
        halt("Cannot set joint angles: " + string(e.what()), __FUNCTION__);
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
 * @brief Initialize footsteps and NAO model.
 * @attention Hardcoded parameters.
 */
void oru_walk::initWMG_NaoModel()
{
    wmg = new WMG();
    wmg->init(wp.preview_window_size);     // size of the preview window

    // each step is defined relatively to the previous step
    double step_x = wp.step_length; // relative X position
    double step_y = 0.1;            // relative Y position

    double ds_constraint[4] = {
        wmg->def_ss_constraint[0],
        wmg->def_ss_constraint[1] + 0.5*step_y,
        wmg->def_ss_constraint[2],
        wmg->def_ss_constraint[3] + 0.5*step_y};


    wmg->AddFootstep(0.0, step_y/2, 0.0, 0, 0, wmg->def_ss_constraint, FS_TYPE_SS_L);

    // Initial double support
    wmg->AddFootstep(0.0, -step_y/2, 0.0, wp.ss_number/2, wp.ss_number/2, ds_constraint, FS_TYPE_DS);


    // all subsequent steps have normal feet size
    // 2 reference ZMP positions in single support 
    // 1 in double support
    // 1 + 2 = 3
    wmg->AddFootstep(0.0   , -step_y/2, 0.0 , wp.ss_number,  wp.ss_number + wp.ds_number, wmg->def_ss_constraint);
    wmg->AddFootstep(step_x,  step_y, 0.0);

    for (int i = 0; i < wp.step_pairs_number; i++)
    {
        wmg->AddFootstep(step_x, -step_y, 0.0);
        wmg->AddFootstep(step_x,  step_y, 0.0);
    }

    // here we give many reference points, since otherwise we 
    // would not have enough steps in preview window to reach 
    // the last footsteps
    wmg->AddFootstep(0.0   , -step_y/2, 0.0, 5*wp.ss_number, 5*wp.ss_number, ds_constraint, FS_TYPE_DS);
    wmg->AddFootstep(0.0   , -step_y/2, 0.0 , 0,  0, wmg->def_ss_constraint, FS_TYPE_SS_R);


// Nao
    readSensors(nao.state_sensor);

    // support foot position and orientation
    nao.init (
            IGM_SUPPORT_LEFT,
            0.0, 0.05, 0.0, // position
            0.0, 0.0, 0.0);  // orientation
    
    wmg->init_param (     
            wp.preview_sampling_time_sec, // sampling time in seconds
            nao.CoM_position[2],          // height of the center of mass
            wp.step_height);              // step height (for interpolation of feet movements)

    double pos_error[POSITION_VECTOR_SIZE];
    nao.state_sensor.getSwingFootPosition (pos_error);
    pos_error[0] =  0.0  - pos_error[0];
    pos_error[1] = -step_y/2 - pos_error[1];
    pos_error[2] =  0.0;//  - pos_error[2];
    wmg->correctNextSSPosition (pos_error);

    wmg->initABMatrices (wp.control_sampling_time_sec);
    wmg->init_state.set (nao.CoM_position[0], nao.CoM_position[1]);
}



/**
 * @brief Solve the MPC problem.
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
