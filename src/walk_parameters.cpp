/**
 * @file
 * @author Alexander Sherikov
 */

#include "walk_parameters.h"



/**
 * @brief Initialize parameters to default values.
 *
 * @param[in] broker parent broker.
 */
walkParameters::walkParameters(ALPtr<ALBroker> broker) :
    pref_proxy(broker)
{
// CoM position feedback
    /**
     * @ref AldNaoPaper "0.5 in the publication by Aldebaran-Robotics"
     */
    feedback_gain = 0.3;
    /**
     * @ref AldNaoPaper "0.003 in the publication by Aldebaran-Robotics"
     * Note, that even though they derive the value of this parameter in 
     * the paper, we simply tuned it.
     */
    feedback_threshold = 0.006;


// parameters of the MPC solver
    mpc_alpha = 1.0;    // penalty for the velocity
    mpc_beta = 4000.0;  // closeness to the reference ZMP points 
    mpc_gamma = 1.0;    // penalty for the jerk
    mpc_regularization = 0.01;
    mpc_tolerance = 1e-7;


// parameters of the walking pattern generator
    preview_window_size = 40;
    preview_sampling_time_ms = 40;
    preview_sampling_time_sec = (double) preview_sampling_time_ms / 1000;

    ss_time_ms = 400;
    ds_time_ms = 40;
    ds_number = 3;
    step_pairs_number = 4;


// parameters of the steps
    /**
     * @ref AldNaoPaper "0.015 in the publication by Aldebaran-Robotics"
     * 0.02 is used in the built-in module, but is is slightly higher when 
     * executed on the robot.
     */
    step_height = 0.02;

    // The longest step in the built-in module.
    step_length = 0.04;

    // assume that z coordinate of a foot after position correction is 0.0
    set_support_z_to_zero = true;


// parameters of the control thread    
    /*
     * Accordingly to the NAOqi documentation
     * @verbatim
        Only DCM has a real-time thread in NAOqi. ::
            scheduling policy: SCHED_FIFO
            scheduling priority: 90
       @endverbatim
     * Cannot set priority >= 70.
     */
    walk_control_thread_priority = 65; // constant

    dcm_time_shift_ms = 2;
    dcm_sampling_time_ms = 10; // constant

    control_sampling_time_ms = 20; // constant
    control_sampling_time_sec = (double) control_sampling_time_ms / 1000;

    loop_time_limit_ms = 15; // less than control_sampling_time_ms




// list of parameters, that are stored in config file:
    param_names.arraySetSize(NUM_PARAMETERS);

    param_names[FEEDBACK_GAIN]            = "feedback_gain";
    param_names[FEEDBACK_THRESHOLD]       = "feedback_threshold";

    param_names[MPC_ALPHA]                = "mpc_alpha";
    param_names[MPC_BETA]                 = "mpc_beta";
    param_names[MPC_GAMMA]                = "mpc_gamma";
    param_names[MPC_REGULARIZATION]       = "mpc_regularization";
    param_names[MPC_TOLERANCE]            = "mpc_tolerance";

    param_names[STEP_HEIGHT]              = "step_height";
    param_names[STEP_LENGTH]              = "step_length";

    param_names[LOOP_TIME_LIMIT_MS]       = "loop_time_limit_ms";
    param_names[DCM_TIME_SHIFT_MS]        = "dcm_time_shift_ms";
    param_names[PREVIEW_SAMPLING_TIME_MS] = "preview_sampling_time_ms";
    param_names[PREVIEW_WINDOW_SIZE]      = "preview_window_size";     

    param_names[SS_CONTROL_LOOPS]         = "ss_control_loops";     
    param_names[DS_CONTROL_LOOPS]         = "ds_control_loops";     
    param_names[DS_NUMBER]                = "ds_number";     
    param_names[STEP_PAIRS_NUMBER]        = "step_pairs_number";     
}



/**
 * @brief Read parameters from configuration file; if the file does
 *  not exist, write the default values to it.
 */
void walkParameters::readParameters()
{
    ALValue preferences;

    try
    {
        preferences = pref_proxy.readPrefFile ("oru_walk", false);
    }
    catch (const ALError &e)
    {
        qiLogInfo ("module.oru_walk") << e.what();
        writeParameters ();
        return;
    }


    if (!preferences.isArray())
    {
        return;
    }


    for (int i = 0; i < preferences.getSize(); i++)
    {
        for (int j = 0; j < NUM_PARAMETERS; j++)
        {
            if (preferences[i][0] != param_names[j])
            {
                continue;
            }

            if (preferences[i][2].isFloat())
            {
                switch (j)
                {
                    case FEEDBACK_GAIN:
                        feedback_gain = preferences[i][2];
                        break;
                    case FEEDBACK_THRESHOLD:
                        feedback_threshold = preferences[i][2];
                        break;
                    case MPC_ALPHA:
                        mpc_alpha = preferences[i][2];
                        break;
                    case MPC_BETA:
                        mpc_beta = preferences[i][2];
                        break;
                    case MPC_GAMMA:
                        mpc_gamma = preferences[i][2];
                        break;
                    case MPC_REGULARIZATION:
                        mpc_regularization = preferences[i][2];
                        break;
                    case MPC_TOLERANCE:
                        mpc_tolerance = preferences[i][2];
                        break;
                    case STEP_HEIGHT:
                        step_height = preferences[i][2];
                        break;
                    case STEP_LENGTH:
                        step_length = preferences[i][2];
                        break;
                    default:
                        break;
                }
            }
            if (preferences[i][2].isInt())
            {
                switch (j)
                {
                    case LOOP_TIME_LIMIT_MS:
                        loop_time_limit_ms = preferences[i][2];
                        break;
                    case DCM_TIME_SHIFT_MS:
                        dcm_time_shift_ms = preferences[i][2];
                        break;
                    case PREVIEW_SAMPLING_TIME_MS:
                        preview_sampling_time_ms = preferences[i][2];
                        control_sampling_time_sec = (double) control_sampling_time_ms / 1000;
                        break;
                    case PREVIEW_WINDOW_SIZE:
                        preview_window_size = preferences[i][2];
                        break;
                    case SS_CONTROL_LOOPS:
                        ss_time_ms = control_sampling_time_ms * (int) preferences[i][2];
                        break;
                    case DS_CONTROL_LOOPS:
                        ds_time_ms = control_sampling_time_ms * (int) preferences[i][2];
                        break;
                    case DS_NUMBER:
                        ds_number = preferences[i][2];     
                        break;
                    case STEP_PAIRS_NUMBER:
                        step_pairs_number = preferences[i][2];     
                        break;
                    default:
                        break;
                }
            }
            break;
        }
    }
}



/**
 * @brief Write the values of parameters to the config file.
 */
void walkParameters::writeParameters()
{
    ALValue preferences;

    preferences.arraySetSize(NUM_PARAMETERS);
    for (int i = 0; i < NUM_PARAMETERS; i++)
    {
        preferences[i].arraySetSize(3);
        preferences[i][0] = param_names[i];
    }


    // descriptions
    preferences[FEEDBACK_GAIN][1]            = "";
    preferences[FEEDBACK_THRESHOLD][1]       = "";

    preferences[MPC_ALPHA][1]                = "";
    preferences[MPC_BETA][1]                 = "";
    preferences[MPC_GAMMA][1]                = "";
    preferences[MPC_REGULARIZATION][1]       = "";
    preferences[MPC_TOLERANCE][1]            = "";

    preferences[STEP_HEIGHT][1]              = "";
    preferences[STEP_LENGTH][1]              = "";

    preferences[LOOP_TIME_LIMIT_MS][1]       = "";
    preferences[DCM_TIME_SHIFT_MS][1]        = "";
    preferences[PREVIEW_SAMPLING_TIME_MS][1] = "";
    preferences[PREVIEW_WINDOW_SIZE][1]      = "";     

    preferences[SS_CONTROL_LOOPS][1]         = "";
    preferences[DS_CONTROL_LOOPS][1]         = "";
    preferences[DS_NUMBER][1]                = "";     
    preferences[STEP_PAIRS_NUMBER][1]        = "";     


    // values
    preferences[FEEDBACK_GAIN][2]               = feedback_gain;
    preferences[FEEDBACK_THRESHOLD][2]          = feedback_threshold;

    preferences[MPC_ALPHA][2]                   = mpc_alpha;
    preferences[MPC_BETA][2]                    = mpc_beta;
    preferences[MPC_GAMMA][2]                   = mpc_gamma;
    preferences[MPC_REGULARIZATION][2]          = mpc_regularization;
    preferences[MPC_TOLERANCE][2]               = mpc_tolerance;

    preferences[STEP_HEIGHT][2]                 = step_height;
    preferences[STEP_LENGTH][2]                 = step_length;

    preferences[LOOP_TIME_LIMIT_MS][2]          = loop_time_limit_ms;
    preferences[DCM_TIME_SHIFT_MS][2]           = dcm_time_shift_ms;
    preferences[PREVIEW_SAMPLING_TIME_MS][2]    = preview_sampling_time_ms;
    preferences[PREVIEW_WINDOW_SIZE][2]         = preview_window_size;

    preferences[SS_CONTROL_LOOPS][2]            = ss_time_ms / control_sampling_time_ms;
    preferences[DS_CONTROL_LOOPS][2]            = ds_time_ms / control_sampling_time_ms;
    preferences[DS_NUMBER][2]                   = ds_number;
    preferences[STEP_PAIRS_NUMBER][2]           = step_pairs_number;


    try
    {
        pref_proxy.writePrefFile("oru_walk", preferences, true); 
    }
    catch (const ALError &e)
    {
        qiLogInfo ("module.oru_walk") << e.what();
    }
}
