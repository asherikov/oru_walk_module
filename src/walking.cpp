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


    // initialize Nao model
    readSensors(nao.state_sensor);
    for (int i = 0; i < LOWER_JOINTS_NUM; i++)
    {
        ref_joint_angles[i] = nao.state_sensor.q[i];
    }


    // start walk control thread
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
}



/**
 * @brief Update joint angles.
 */
void oru_walk::readSensors(jointState& joint_state)
{
    vector<float> sensorValues;

    access_sensor_values->GetValues (sensorValues);
    for (int i = 0; i < JOINTS_NUM; i++)
    {
        joint_state.q[i] = sensorValues[i];
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
 * @brief The position of the next support foot may change from the targeted, 
 * this function moves it to the right place.
 */
void oru_walk::correctNextSupportPosition(WMG &wmg)
{
    Transform<double,3> swing_foot_posture;
    nao.getSwingFootPosture (nao.state_sensor, swing_foot_posture.data());
    wmg.changeNextSSPosition (swing_foot_posture.data(), wp.set_support_z_to_zero);
}



/**
 * @brief A control loop, that is executed in separate thread.
 * @attention REAL-TIME!
 */
void oru_walk::walkControl()
{
    oruw_timer timer(__FUNCTION__, wp.loop_time_limit_ms);


    smpc::solver solver(
            wp.preview_window_size,
            wp.mpc_alpha,
            wp.mpc_beta,
            wp.mpc_gamma,
            wp.mpc_regularization,
            wp.mpc_tolerance);
//    solver.enable_fexceptions();


    WMG wmg(wp.preview_window_size,
            wp.preview_sampling_time_ms,
            wp.step_height);
    wmg.T_ms[0] = wp.control_sampling_time_ms;
    wmg.T_ms[1] = wp.control_sampling_time_ms;

    try
    {
        // steps
        initWalkPattern(wmg);
        // error in position of the swing foot
        correctNextSupportPosition(wmg);
    }
    catch (...)
    {
        return;
    }


    nao.getCoM (nao.state_sensor, nao.CoM_position);
    smpc_parameters mpc(wp.preview_window_size, nao.CoM_position[2]);
    mpc.init_state.set (nao.CoM_position[0], nao.CoM_position[1]);


    jointState target_joint_state = nao.state_model;
    for (;;)
    {
        boost::unique_lock<boost::mutex> lock(walk_control_mutex);
        walk_control_condition.wait(lock);
        lock.unlock();


        timer.reset();


        ORUW_LOG_JOINTS(nao.state_sensor, target_joint_state);
        ORUW_LOG_COM(mpc, nao);
        ORUW_LOG_FEET(nao);


        try
        {
            feedbackError (mpc.init_state);

            if (solveMPCProblem (wmg, mpc, solver))  // solve MPC
            {
                if (wmg.isSupportSwitchNeeded())
                {
                    correctNextSupportPosition(wmg);
                    nao.switchSupportFoot();
                }

                // the old solution from is an initial guess;
                solveIKsendCommands (mpc, solver, 1, wmg);
                target_joint_state = nao.state_model;
                solveIKsendCommands (mpc, solver, 2, wmg);
            }
            else
            {
                break;
            }

            if (!timer.check()) 
            {
                halt("Time limit is violated!\n", __FUNCTION__);
            }
        }
        catch (...)
        {
            break;
        }
    }

    ORUW_LOG_STEPS(wmg);
    ORUW_LOG_CLOSE;
}



/**
 * @brief Solve inverse kinematics and send commands to the controllers.
 *
 * @param[in] callback_start_time_ms the time, when the callback was started.
 * @param[in] control_loop_num number of control loops in future (>= 1).
 */
void oru_walk::solveIKsendCommands (
        const smpc_parameters &mpc,
        const smpc::solver &solver,
        const int control_loop_num,
        WMG &wmg)
{
    smpc::state_orig CoM;
    CoM.get_state(solver, control_loop_num-1);

    // hCoM is constant!
    nao.setCoM(CoM.x(), CoM.y(), mpc.hCoM);


    // support foot and swing foot position/orientation
    wmg.getFeetPositions (
            control_loop_num * wp.control_sampling_time_ms, 
            nao.left_foot_posture->data(), 
            nao.right_foot_posture->data());


    // inverse kinematics
    int iter_num = nao.igm (
            ref_joint_angles, 
            wp.igm_mu, 
            wp.igm_tol, 
            wp.igm_max_iter);
    ORUW_LOG_MESSAGE("IGM iterations num: %d\n", iter_num);
    if (iter_num < 0)
    {
        halt("IK does not converge.\n", __FUNCTION__);
    }
    int failed_joint = nao.state_model.checkJointBounds();
    if (failed_joint >= 0)
    {
        ORUW_LOG_MESSAGE("Failed joint: %d\n", failed_joint);
        halt("Joint bounds are violated.\n", __FUNCTION__);
    }


    // Set commands
    try
    {
        joint_commands[4][0] = last_dcm_time_ms + control_loop_num * wp.control_sampling_time_ms;
        for (int i = 0; i < LOWER_JOINTS_NUM; i++)
        {
            joint_commands[5][i][0] = nao.state_model.q[i];
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
void oru_walk::feedbackError (smpc::state_orig &init_state)
{
    double CoM_pos[POSITION_VECTOR_SIZE];
    nao.getCoM (nao.state_sensor, CoM_pos);


    smpc::state_orig state_error;
    state_error.set (
            init_state.x() - CoM_pos[0],
            init_state.y() - CoM_pos[1]);

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

    init_state.x() -= wp.feedback_gain * state_error.x();
    init_state.y() -= wp.feedback_gain * state_error.y();
}



/**
 * @brief Solve the MPC problem.
 *
 * @return false if there is not enough steps, true otherwise.
 */
bool oru_walk::solveMPCProblem (
        WMG &wmg,
        smpc_parameters &mpc,
        smpc::solver &solver)
{
    oruw_timer timer(__FUNCTION__, wp.loop_time_limit_ms);

    if (wmg.formPreviewWindow(mpc) == WMG_HALT)
    {
        stopWalking("Not enough steps to form preview window. Stopping.");
        return (false);
    }


    //------------------------------------------------------
    solver.set_parameters (mpc.T, mpc.h, mpc.h[0], mpc.angle, mpc.zref_x, mpc.zref_y, mpc.lb, mpc.ub);
    solver.form_init_fp (mpc.fp_x, mpc.fp_y, mpc.init_state, mpc.X);
    int num_iq_constr = solver.solve();
    mpc.init_state.get_next_state(solver);
    //------------------------------------------------------


    ORUW_LOG_MESSAGE("Num of active constraints: %d\n", num_iq_constr);
    if (!timer.check()) 
    {
        halt("Time limit is violated!\n", __FUNCTION__);
    }
    return (true);
}


// ==============================================================================


/**
 * @brief Unregister callback and log a message.
 *
 * @param[in] message a message
 */
void oru_walk::stopWalking(const char* message)
{
    ORUW_LOG_MESSAGE("%s", message);
    qiLogInfo ("module.oru_walk") << message;
    dcm_callback_connection.disconnect();
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
 * @brief An interface function that is called remotely to stop
 * the execution.
 */
void oru_walk::stopWalkingRemote()
{
    stopWalking ("Stopped by user's request.\n");
    ORUW_LOG_CLOSE;
}
