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
    mpc_alpha = 400.0;  // penalty for the velocity
    mpc_beta = 4000.0;  // closeness to the reference ZMP points 
    mpc_gamma = 1.0;    // penalty for the jerk
    mpc_regularization = 0.01;
    mpc_tolerance = 1e-7;


// parameters of the walking pattern generator
    preview_window_size = 40;
    preview_sampling_time_ms = 40;
    preview_sampling_time_sec = (double) preview_sampling_time_ms / 1000;

    ss_number = 10;
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

    loop_time_limit_ms = 18000; // less than control_sampling_time_ms




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

    param_names[SS_NUMBER]                = "ss_number";     
    param_names[DS_NUMBER]                = "ds_number";     
    param_names[STEP_PAIRS_NUMBER]        = "step_pairs_number";     


    param_description.arraySetSize(NUM_PARAMETERS);

    param_description[FEEDBACK_GAIN]            = "";
    param_description[FEEDBACK_THRESHOLD]       = "";

    param_description[MPC_ALPHA]                = "";
    param_description[MPC_BETA]                 = "";
    param_description[MPC_GAMMA]                = "";
    param_description[MPC_REGULARIZATION]       = "";
    param_description[MPC_TOLERANCE]            = "";

    param_description[STEP_HEIGHT]              = "";
    param_description[STEP_LENGTH]              = "";

    param_description[LOOP_TIME_LIMIT_MS]       = "";
    param_description[DCM_TIME_SHIFT_MS]        = "";
    param_description[PREVIEW_SAMPLING_TIME_MS] = "";
    param_description[PREVIEW_WINDOW_SIZE]      = "";     

    param_description[SS_NUMBER]                = "";     
    param_description[DS_NUMBER]                = "";     
    param_description[STEP_PAIRS_NUMBER]        = "";     
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
                    case SS_NUMBER:
                        ss_number = preferences[i][2];
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
        preferences[i][1] = param_description[i];
        switch (i)
        {
            case FEEDBACK_GAIN:          
                preferences[i][2] = feedback_gain;
                break;
            case FEEDBACK_THRESHOLD:     
                preferences[i][2] = feedback_threshold;
                break;
            case MPC_ALPHA:              
                preferences[i][2] = mpc_alpha;
                break;
            case MPC_BETA:               
                preferences[i][2] = mpc_beta;
                break;
            case MPC_GAMMA:              
                preferences[i][2] = mpc_gamma;
                break;
            case MPC_REGULARIZATION:     
                preferences[i][2] = mpc_regularization;
                break;
            case MPC_TOLERANCE:          
                preferences[i][2] = mpc_tolerance;
                break;
            case STEP_HEIGHT:            
                preferences[i][2] = step_height;
                break;
            case STEP_LENGTH:            
                preferences[i][2] = step_length;
                break;
            case LOOP_TIME_LIMIT_MS:     
                preferences[i][2] = loop_time_limit_ms;
                break;
            case DCM_TIME_SHIFT_MS:     
                preferences[i][2] = dcm_time_shift_ms;
                break;
            case PREVIEW_SAMPLING_TIME_MS:
                preferences[i][2] = preview_sampling_time_ms;
                break;
            case PREVIEW_WINDOW_SIZE:    
                preferences[i][2] = preview_window_size;
                break;
            case SS_NUMBER:
                preferences[i][2] = ss_number;
                break;
            case DS_NUMBER:
                preferences[i][2] = ds_number;
                break;
            case STEP_PAIRS_NUMBER:
                preferences[i][2] = step_pairs_number;
                break;
            default:
                preferences[i][2] = 0;
                break;
        }
    }


    try
    {
        pref_proxy.writePrefFile("oru_walk", preferences, true); 
    }
    catch (const ALError &e)
    {
        qiLogInfo ("module.oru_walk") << e.what();
    }
}
