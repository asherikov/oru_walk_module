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

    joint_feedback_gain = 0.0;

    mpc_alpha = 400.0;  // penalty for the velocity
    mpc_beta = 4000.0;  // closeness to the reference ZMP points 
    mpc_gamma = 1.0;    // penalty for the jerk
    mpc_regularization = 0.01;
    mpc_tolerance = 1e-7;


    /**
     * 0.0135 in the old version of our module.
     * @ref AldNaoPaper "0.015 in the publication by Aldebaran-Robotics"
     * 0.02 is used in the built-in module, but is is slightly higher when 
     * executed on the robot.
     */
    step_height = 0.02;
    // The longest step in the built-in module.
    step_length = 0.04;

    control_sampling_time_ms = 20; // constant
    control_sampling_time_sec = (double) control_sampling_time_ms / 1000;
    loop_time_limit_ms = 18000; // less than 20ms
    preview_sampling_time_ms = 40;
    preview_sampling_time_sec = (double) preview_sampling_time_ms / 1000;

    preview_window_size = 40;


    filter_window_length = 1; // not used


    ss_number = 11;
    ds_number = 3;
    step_pairs_number = 8;


    param_names.arraySetSize(NUM_PARAMETERS);
    param_names[FEEDBACK_GAIN]            = "feedback_gain";
    param_names[FEEDBACK_THRESHOLD]       = "feedback_threshold";
    param_names[JOINT_FEEDBACK_GAIN]      = "joint_feedback_gain";
    param_names[MPC_ALPHA]                = "mpc_alpha";
    param_names[MPC_BETA]                 = "mpc_beta";
    param_names[MPC_GAMMA]                = "mpc_gamma";
    param_names[MPC_REGULARIZATION]       = "mpc_regularization";
    param_names[MPC_TOLERANCE]            = "mpc_tolerance";
    param_names[STEP_HEIGHT]              = "step_height";
    param_names[STEP_LENGTH]              = "step_length";
    param_names[LOOP_TIME_LIMIT_MS]       = "loop_time_limit_ms";
    param_names[PREVIEW_SAMPLING_TIME_MS] = "preview_sampling_time_ms";
    param_names[PREVIEW_WINDOW_SIZE]      = "preview_window_size";     
    param_names[SS_NUMBER]                = "ss_number";     
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
                    case JOINT_FEEDBACK_GAIN:
                        joint_feedback_gain = preferences[i][2];
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
        switch (i)
        {
            case FEEDBACK_GAIN:          
                preferences[i][1] = string("");
                preferences[i][2] = feedback_gain;
                break;
            case FEEDBACK_THRESHOLD:     
                preferences[i][1] = string("");
                preferences[i][2] = feedback_threshold;
                break;
            case JOINT_FEEDBACK_GAIN:    
                preferences[i][1] = string("");
                preferences[i][2] = joint_feedback_gain;
                break;
            case MPC_ALPHA:              
                preferences[i][1] = string("");
                preferences[i][2] = mpc_alpha;
                break;
            case MPC_BETA:               
                preferences[i][1] = string("");
                preferences[i][2] = mpc_beta;
                break;
            case MPC_GAMMA:              
                preferences[i][1] = string("");
                preferences[i][2] = mpc_gamma;
                break;
            case MPC_REGULARIZATION:     
                preferences[i][1] = string("");
                preferences[i][2] = mpc_regularization;
                break;
            case MPC_TOLERANCE:          
                preferences[i][1] = string("");
                preferences[i][2] = mpc_tolerance;
                break;
            case STEP_HEIGHT:            
                preferences[i][1] = string("");
                preferences[i][2] = step_height;
                break;
            case STEP_LENGTH:            
                preferences[i][1] = string("");
                preferences[i][2] = step_length;
                break;
            case LOOP_TIME_LIMIT_MS:     
                preferences[i][1] = string("");
                preferences[i][2] = loop_time_limit_ms;
                break;
            case PREVIEW_SAMPLING_TIME_MS:
                preferences[i][1] = string("");
                preferences[i][2] = preview_sampling_time_ms;
                break;
            case PREVIEW_WINDOW_SIZE:    
                preferences[i][1] = string("");
                preferences[i][2] = preview_window_size;
                break;
            case SS_NUMBER:
                preferences[i][1] = string("");
                preferences[i][2] = ss_number;
                break;
            case DS_NUMBER:
                preferences[i][1] = string("");
                preferences[i][2] = ds_number;
                break;
            case STEP_PAIRS_NUMBER:
                preferences[i][1] = string("");
                preferences[i][2] = step_pairs_number;
                break;
            default:
                preferences[i][1] = string("");
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
