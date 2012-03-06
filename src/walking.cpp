/**
 * @file
 * @author Antonio Paolillo
 * @author Dimitar Dimitrov
 * @author Alexander Sherikov
 */

#include "oru_walk.h"
#include "oruw_log.h"
#include "oruw_timer.h"


/**
 * @brief Construct and initialize all necessary classes and register 
 * callback function.
 */
void oru_walk::walk()
{
    ORUW_LOG_OPEN;
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


// solver    
    solver = new smpc::solver(
            wp.preview_window_size,
            wp.mpc_alpha,
            wp.mpc_beta,
            wp.mpc_gamma,
            wp.mpc_regularization,
            wp.mpc_tolerance);
    solver->enable_fexceptions();


// models
    initWMG_NaoModel();


// register callback
    dcm_loop_counter = 0;
    try
    {
        dcm_callback_connection =
            getParentBroker()->getProxy("DCM")->getModule()->atPostProcess
            (boost::bind(&oru_walk::dcmCallback, this));
    }
    catch (const ALError &e)
    {
        ORUW_LOG_MESSAGE("Callback registration failed: %s\n", e.what());
        halt("Callback registration failed!", __FUNCTION__);
    }


    try
    {
        boost::thread walk_control_thread(&oru_walk::walkControl, this);
        struct sched_param walk_control_thread_sched;

        walk_control_thread_sched.sched_priority = wp.walk_control_thread_priority;
        int retval = pthread_setschedparam(
                walk_control_thread.native_handle(), 
                SCHED_FIFO, 
                &walk_control_thread_sched);
        if (retval != 0)
        {
            // Assume that this error is not critical
            ORUW_LOG_MESSAGE("Cannot change the priority of the walk control thread: %s\n", strerror(retval));
        }

        walk_control_thread.detach();
    }
    catch (...)
    {
        halt("Failed to spawn the walk control thread.\n", __FUNCTION__);
    }
}


/**
 * @brief Wake up walk control thread periodically.
 */
void oru_walk::dcmCallback()
{
    dcm_loop_counter++;
    if (dcm_loop_counter % (wp.control_sampling_time_ms / wp.dcm_sampling_time_ms) == 0)
    {
        last_dcm_time_ms = *last_dcm_time_ms_ptr + wp.dcm_time_shift_ms;
        readSensors (nao.state_sensor);

        boost::mutex::scoped_lock lock(walk_control_mutex);
        walk_control_condition.notify_one();
        lock.unlock();
    }
}



/**
 * @brief Log a message, remove stiffness and die.
 *
 * @param[in] message a message
 * @param[in] function name of the calling function.
 */
void oru_walk::halt(const char *message, const char* function)
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
void oru_walk::stopWalking(const char* message)
{
    ORUW_LOG_STEPS(wmg);
    ORUW_LOG_MESSAGE("%s", message);
    ORUW_LOG_CLOSE;

    qiLogInfo ("module.oru_walk") << message;
    dcm_callback_connection.disconnect();
}



/**
 * @brief An interface function that is called remotely to stop
 * the execution.
 */
void oru_walk::stopWalkingRemote()
{
    stopWalking ("Stopped by user's request.\n");
}



/**
 * @brief A control loop, that is executed in separate thread.
 * @attention REAL-TIME!
 */
void oru_walk::walkControl()
{
    oruw_timer timer(__FUNCTION__, wp.loop_time_limit_ms);


    for (;;)
    {
        boost::unique_lock<boost::mutex> lock(walk_control_mutex);
        walk_control_condition.wait(lock);
        lock.unlock();


        timer.reset();


        ORUW_LOG_JOINTS(nao.state_sensor, nao.state_model);
        ORUW_LOG_COM(mpc, nao);
        ORUW_LOG_FEET(nao);


        try
        {
            feedbackError ();

            if (solveMPCProblem ())  // solve MPC
            {
                if (wmg->isSupportSwitchNeeded())
                {
                    wmg->changeNextSSPosition(nao.switchSupportFoot(), wp.set_support_z_to_zero);
                    nao_next.support_foot = nao.support_foot;
                }

                nao.state_model = nao_next.state_model; // the old solution from nao_next -> initial guess;
                solveIKsendCommands (last_dcm_time_ms, 1, nao);

                nao_next.state_model = nao.state_model;
                solveIKsendCommands (last_dcm_time_ms, 2, nao_next);
            }
            else
            {
                return;
            }

            if (!timer.check()) 
            {
                halt("Time limit is violated!\n", __FUNCTION__);
            }
        }
        catch (...)
        {
            return;
        }
    }
}



/**
 * @brief Solve inverse kinematics and send commands to the controllers.
 *
 * @param[in] callback_start_time_ms the time, when the callback was started.
 * @param[in] control_loop_num number of control loops in future (>= 1).
 * @param[in,out] nao_model model of the nao.
 */
void oru_walk::solveIKsendCommands (
        const int callback_start_time_ms,
        const int control_loop_num,
        nao_igm &nao_model)
{
    smpc::state_orig CoM;
    CoM.get_state(*solver, control_loop_num-1);

    // hCoM is constant!
    nao_model.setCoM(CoM.x(), CoM.y(), mpc->hCoM);


    // support foot and swing foot position/orientation
    wmg->getFeetPositions (
            control_loop_num * wp.control_sampling_time_ms, 
            nao_model.left_foot_posture->data(), 
            nao_model.right_foot_posture->data());


    // inverse kinematics
    int iter_num = nao_model.igm ();    
    ORUW_LOG_MESSAGE("IGM iterations num: %d\n", iter_num);
    if (iter_num < 0)
    {
        halt("IK does not converge.\n", __FUNCTION__);
    }
    int failed_joint = nao_model.state_model.checkJointBounds();
    if (failed_joint >= 0)
    {
        ORUW_LOG_MESSAGE("Failed joint: %d\n", failed_joint);
        halt("Joint bounds are violated.\n", __FUNCTION__);
    }


    // Set commands
    try
    {
        joint_commands[4][0] = callback_start_time_ms + control_loop_num * wp.control_sampling_time_ms;
        for (int i = 0; i < LOWER_JOINTS_NUM; i++)
        {
            joint_commands[5][i][0] = nao_model.state_model.q[i];
        }
        dcm_proxy->setAlias(joint_commands);
    }
    catch (const AL::ALError &e)
    {
        ORUW_LOG_MESSAGE("Cannot set joint angles: %s", e.what());
        halt("Cannot set joint angles!", __FUNCTION__);
    }
}



/**
 * @brief Correct state and the model based on the sensor data.
 */
void oru_walk::feedbackError ()
{
    double CoM_pos[POSITION_VECTOR_SIZE];
    nao.getCoM (nao.state_sensor, CoM_pos);


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
void oru_walk::readSensors(jointState& nao_state)
{
    vector<float> sensorValues;

    access_sensor_values->GetValues (sensorValues);
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
    nao_next = nao;
    

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

    wmg->setFootstepDefaults(0, 0, 0, wmg->def_ss_constraint);
    wmg->addFootstep(0.0, step_y/2, 0.0, FS_TYPE_SS_L);

    // Initial double support
    wmg->setFootstepDefaults(3*wp.ss_time_ms, 0, 0, ds_constraint);
    wmg->addFootstep(0.0, -step_y/2, 0.0, FS_TYPE_DS);


    // all subsequent steps have normal feet size
    wmg->setFootstepDefaults(wp.ss_time_ms, 0, 0, wmg->def_ss_constraint);
    wmg->addFootstep(0.0   , -step_y/2, 0.0);
    wmg->setFootstepDefaults(wp.ss_time_ms, wp.ds_time_ms, wp.ds_number);
    wmg->addFootstep(step_x,  step_y,   0.0);

    for (int i = 0; i < wp.step_pairs_number; i++)
    {
        wmg->addFootstep(step_x, -step_y, 0.0);
        wmg->addFootstep(step_x,  step_y, 0.0);
    }

    // here we give many reference points, since otherwise we 
    // would not have enough steps in preview window to reach 
    // the last footsteps
    wmg->setFootstepDefaults(5*wp.ss_time_ms, 0, 0, ds_constraint);
    wmg->addFootstep(0.0   , -step_y/2, 0.0, FS_TYPE_DS);
    wmg->setFootstepDefaults(0, 0, 0, wmg->def_ss_constraint);
    wmg->addFootstep(0.0   , -step_y/2, 0.0, FS_TYPE_SS_R);


// error in position of the swing foot    
    nao.getSwingFootPosture (nao.state_sensor);
    wmg->changeNextSSPosition (nao.swing_foot_posture->data(), wp.set_support_z_to_zero);
}



/**
 * @brief Solve the MPC problem.
 *
 * @return false if there is not enough steps, true otherwise.
 */
bool oru_walk::solveMPCProblem ()
{
    oruw_timer timer(__FUNCTION__, wp.loop_time_limit_ms);

    if (wmg->formPreviewWindow(*mpc) == WMG_HALT)
    {
        stopWalking("Not enough steps to form preview window. Stopping.");
        return (false);
    }


    //------------------------------------------------------
    solver->set_parameters (mpc->T, mpc->h, mpc->h[0], mpc->angle, mpc->zref_x, mpc->zref_y, mpc->lb, mpc->ub);
    solver->form_init_fp (mpc->fp_x, mpc->fp_y, mpc->init_state, mpc->X);
    int num_iq_constr = solver->solve();
    mpc->init_state.get_next_state(*solver);
    //------------------------------------------------------


    ORUW_LOG_MESSAGE("Num of active constraints: %d\n", num_iq_constr);
    if (!timer.check()) 
    {
        halt("Time limit is violated!\n", __FUNCTION__);
    }
    return (true);
}
