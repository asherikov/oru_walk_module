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

    if (mpc != NULL)
    {
        delete mpc;
        mpc = NULL;
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
    int current_time_ms = dcmProxy->getTime(0);
    walkCommands[4][0] = current_time_ms + wp.control_sampling_time_ms;
    walkCommands[4][1] = current_time_ms + 2*wp.control_sampling_time_ms;

    readSensors (nao.state_sensor);


    ORUW_LOG_JOINTS(nao.state_sensor, nao.state_model);
    ORUW_LOG_COM(mpc, nao.state_sensor);
    ORUW_LOG_FEET(nao);
    ORUW_LOG_JOINT_VELOCITIES(nao.state_sensor, wp.control_sampling_time_sec);



    // solve MPC
    feedbackError ();
    if (! solveMPCProblem ())
    {
        return;
    }


    double left_foot_pos[POSITION_VECTOR_SIZE + 1];
    double right_foot_pos[POSITION_VECTOR_SIZE + 1];
    int failed_joint;


    /// @attention hCoM is constant!
    nao.setCoM(mpc->init_state.x(), mpc->init_state.y(), mpc->hCoM);


    // support foot and swing foot position/orientation
    wmg->getFeetPositions (wp.control_sampling_time_ms, left_foot_pos, right_foot_pos);
    nao.setFeetPostures (left_foot_pos, right_foot_pos);


    // inverse kinematics    
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
    nao.setCoM(CoM.x(), CoM.y(), mpc->hCoM);


    // support foot and swing foot position/orientation
    wmg->getFeetPositions (2*wp.control_sampling_time_ms, left_foot_pos, right_foot_pos);
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
            mpc->init_state.x() - CoM_pos[0],
            mpc->init_state.y() - CoM_pos[1]);

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

    mpc->init_state.x() -= wp.feedback_gain * state_error.x();
    mpc->init_state.y() -= wp.feedback_gain * state_error.y();
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
    // each step is defined relatively to the previous step
    double step_x = wp.step_length; // relative X position
    double step_y = 0.1;            // relative Y position


// NAO
    readSensors(nao.state_sensor);

    // support foot position and orientation
    nao.init (
            IGM_SUPPORT_LEFT,
            0.0, 0.05, 0.0, // position
            0.0, 0.0, 0.0);  // orientation
    

//  WMG & smpc_parameters  
    wmg = new WMG(
            wp.preview_window_size,
            wp.preview_sampling_time_ms,  // sampling time in ms
            wp.step_height);              // step height (for interpolation of feet movements)
    wmg->T_ms[0] = wp.control_sampling_time_ms;
    wmg->T_ms[1] = wp.control_sampling_time_ms;
    

    mpc = new smpc_parameters (
            wmg->N,
            nao.CoM_position[2]);         // height of the center of mass
    mpc->init_state.set (nao.CoM_position[0], nao.CoM_position[1]);



// steps
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


// error in position of the swing foot    
    double pos_error[POSITION_VECTOR_SIZE];
    nao.state_sensor.getSwingFootPosition (pos_error);
    pos_error[0] =  0.0  - pos_error[0];
    pos_error[1] = -step_y/2 - pos_error[1];
    pos_error[2] =  0.0;//  - pos_error[2];
    wmg->correctNextSSPosition (pos_error);
}



/**
 * @brief Solve the MPC problem.
 */
bool oru_walk::solveMPCProblem ()
{
    ORUW_TIMER(wp.loop_time_limit_ms);

    if (next_preview_len_ms == 0)
    {
        next_preview_len_ms = wp.preview_sampling_time_ms;
    }

    /// @todo Works only for 20/40!
    wmg->T_ms[2] = next_preview_len_ms;

    if (wmg->isSupportSwitchNeeded())
    {
        double pos_error[POSITION_VECTOR_SIZE];
        nao.switchSupportFoot(pos_error);
        wmg->correctNextSSPosition(pos_error);
    }


    if (wmg->formPreviewWindow(*mpc) == WMG_HALT)
    {
        stopWalking("Not enough steps to form preview window. Stopping.");
        return (false);
    }



    //------------------------------------------------------
    solver->set_parameters (mpc->T, mpc->h, mpc->h[0], mpc->angle, mpc->zref_x, mpc->zref_y, mpc->lb, mpc->ub);
    solver->form_init_fp (mpc->fp_x, mpc->fp_y, mpc->init_state, mpc->X);
    int num_iq_constr = solver->solve();
    ORUW_LOG_MESSAGE("Num of active constraints: %d\n", num_iq_constr);
    mpc->init_state.get_next_state(*solver);
    //------------------------------------------------------
    
    ORUW_TIMER_CHECK;
    return (true);
}
