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
    /// @attention Hardcoded parameters.
    control_sampling_time_ms = 10;
    preview_sampling_time_ms = 20;
    next_preview_len_ms = 0;
    int preview_window_size = 40;
    feedback_gain = 0.2;
    feedback_threshold = 0.008;


// WMG
    initWMG (preview_window_size);


// solver    
    if (solver != NULL)
    {
        delete solver;
    }
    /// @attention Hardcoded parameters.
    solver = new smpc::solver(
            wmg->N, // size of the preview window
            1500.0,  // Alpha
            11000.0,  // Beta
            1.0,    // Gamma
            0.01,   // regularization
            1e-7);  // tolerance


// models
    initNaoModel ();
    wmg->init_param (     
            (double) preview_sampling_time_ms / 1000, // sampling time in seconds
            nao.CoM_position[2],                      // height of the center of mass
            0.015);                // step height (for interpolation of feet movements)
    /// 0.0135 in our version
    /// @ref AldNaoPaper "0.015 in the paper"

    wmg->initABMatrices ((double) control_sampling_time_ms / 1000);
    wmg->init_state.set (nao.CoM_position[0], nao.CoM_position[1]);
    old_state = wmg->init_state;


    ORUW_LOG_OPEN;

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
}



/**
 * @brief 
 * @attention REAL-TIME!
 */
void oru_walk::callbackEveryCycle_walk()
{
    ORUW_TIMER;
    ORUW_LOG_JOINTS(accessSensorValues, accessActuatorValues);
    ORUW_LOG_COM(nao, accessSensorValues);
    ORUW_LOG_SWING_FOOT(nao, accessSensorValues);

    correctStateAndModel ();
    //updateModelJoints ();

    solveMPCProblem ();


    // support foot and swing foot position/orientation
    double swing_foot_pos[POSITION_VECTOR_SIZE];
    double angle;
    wmg->getSwingFootPosition (
            WMG_SWING_2D_PARABOLA, 
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
        walkCommands[4][0] = dcmProxy->getTime(next_preview_len_ms);
        dcmProxy->setAlias(walkCommands);
    }
    catch (const AL::ALError &e)
    {
        throw ALERROR(getName(), __FUNCTION__, "Cannot set joint angles: " + e.toString());
    }

    next_preview_len_ms -= control_sampling_time_ms;
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
    state_sensor.x()  = CoM_pos[0];
    state_sensor.y()  = CoM_pos[1];
    //state_sensor.vx() = (state_sensor.x()  - old_state.x())  / control_sampling_time_ms;
    //state_sensor.ax() = (state_sensor.vx() - old_state.vx()) / control_sampling_time_ms;
    //state_sensor.vy() = (state_sensor.y()  - old_state.y())  / control_sampling_time_ms;
    //state_sensor.ay() = (state_sensor.vy() - old_state.vy()) / control_sampling_time_ms;

    smpc::state_orig state_error;
    state_error.set (
            wmg->init_state.x()  - state_sensor.x(),
            wmg->init_state.y()  - state_sensor.y());
    /*
    state_error.set (
            wmg->init_state.x()  - state_sensor.x(),
            wmg->init_state.vx() - state_sensor.vx(),
            wmg->init_state.ax() - state_sensor.ax(),
            wmg->init_state.y()  - state_sensor.y(),
            wmg->init_state.vy() - state_sensor.vy(),
            wmg->init_state.ay() - state_sensor.ay());
    */

    if (state_error.x() > feedback_threshold)
    {
        state_error.x() -= feedback_threshold;
    }
    else if (state_error.x() < -feedback_threshold)
    {
        state_error.x() += feedback_threshold;
    }
    else
    {
        state_error.x() = 0.0;
    }

    if (state_error.y() > feedback_threshold)
    {
        state_error.y() -= feedback_threshold;
    }
    else if (state_error.y() < -feedback_threshold)
    {
        state_error.y() += feedback_threshold;
    }
    else
    {
        state_error.y() = 0.0;
    }

    wmg->init_state.x()  -= feedback_gain * state_error.x();
    wmg->init_state.y()  -= feedback_gain * state_error.y();
    //wmg->init_state.vx() -= 0;
    //wmg->init_state.ax() -= 0;
    //wmg->init_state.vy() -= 0;
    //wmg->init_state.ay() -= 0;

    old_state = wmg->init_state;
}



/**
 * @brief 
 */
void oru_walk::initNaoModel ()
{
    updateModelJoints();


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
 */
void oru_walk::initWMG (const int preview_window_size)
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
}



/**
 * @brief 
 */
void oru_walk::solveMPCProblem ()
{
    if (next_preview_len_ms == 0)
    {
        bool switch_foot = false;
        WMGret wmg_retval = wmg->FormPreviewWindow(&switch_foot);

        if (wmg_retval == WMG_HALT)
        {
            stopWalking();
            return;
        }

        if (switch_foot)
        {
            nao.switchSupportFoot();
        }

        next_preview_len_ms = preview_sampling_time_ms;
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
}
