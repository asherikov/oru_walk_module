/**
 * @file
 * @author Alexander Sherikov
 */

#include "walk_parameters.h"



walkParameters::walkParameters(ALPtr<ALBroker> broker) :
    pref_proxy(broker)
{
    feedback_gain = 0.3;
    feedback_threshold = 0.006;

    joint_feedback_gain = 0.2;

    mpc_alpha = 400.0;
    mpc_beta = 8000.0;
    mpc_gamma = 1.0;
    mpc_regularization = 0.01;
    mpc_tolerance = 1e-7;


    step_height = 0.013;
    /// 0.0135 in the old version of our module
    /// @ref AldNaoPaper "0.015 in the paper"
    /// 0.02 is used in the built-in module


    control_sampling_time_ms = 10; // constant
    loop_time_limit_ms = 9000; 
    preview_sampling_time_ms = 20;
    preview_window_size = 40;


    filter_window_length = 1;


    param_names.resize(NUM_PARAMETERS);
    param_names[FEEDBACK_GAIN]            = "feedback_gain";
    param_names[FEEDBACK_THRESHOLD]       = "feedback_threshold";
    param_names[JOINT_FEEDBACK_GAIN]      = "joint_feedback_gain";
    param_names[MPC_ALPHA]                = "mpc_alpha";
    param_names[MPC_BETA]                 = "mpc_beta";
    param_names[MPC_GAMMA]                = "mpc_gamma";
    param_names[MPC_REGULARIZATION]       = "mpc_regularization";
    param_names[MPC_TOLERANCE]            = "mpc_tolerance";
    param_names[STEP_HEIGHT]              = "step_height";
    param_names[LOOP_TIME_LIMIT_MS]       = "loop_time_limit_ms";
    param_names[PREVIEW_SAMPLING_TIME_MS] = "preview_sampling_time_ms";
    param_names[PREVIEW_WINDOW_SIZE]      = "preview_window_size";     
}



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


    for (int i = 0; i < preferences.getSize(); i++)
    {
        if (!preferences[i][0].isString())
        {
            continue;
        }

        if (string(preferences[i][0]) == param_names[FEEDBACK_GAIN])
        {
            if (preferences[i][2].isFloat())
            {
                feedback_gain = preferences[i][2];
            }
            continue;
        }

        if (string(preferences[i][0]) == param_names[FEEDBACK_THRESHOLD])
        {
            if (preferences[i][2].isFloat())
            {
                feedback_threshold = preferences[i][2];
            }
            continue;
        }

        if (string(preferences[i][0]) == param_names[JOINT_FEEDBACK_GAIN])
        {
            if (preferences[i][2].isFloat())
            {
                joint_feedback_gain = preferences[i][2];
            }
            continue;
        }

        if (string(preferences[i][0]) == param_names[MPC_ALPHA])
        {
            if (preferences[i][2].isFloat())
            {
                mpc_alpha = preferences[i][2];
            }
            continue;
        }

        if (string(preferences[i][0]) == param_names[MPC_BETA])
        {
            if (preferences[i][2].isFloat())
            {
                mpc_beta = preferences[i][2];
            }
            continue;
        }

        if (string(preferences[i][0]) == param_names[MPC_GAMMA])
        {
            if (preferences[i][2].isFloat())
            {
                mpc_gamma = preferences[i][2];
            }
            continue;
        }

        if (string(preferences[i][0]) == param_names[MPC_REGULARIZATION])
        {
            if (preferences[i][2].isFloat())
            {
                mpc_regularization = preferences[i][2];
            }
            continue;
        }

        if (string(preferences[i][0]) == param_names[MPC_TOLERANCE])
        {
            if (preferences[i][2].isFloat())
            {
                mpc_tolerance = preferences[i][2];
            }
            continue;
        }

        if (string(preferences[i][0]) == param_names[STEP_HEIGHT])
        {
            if (preferences[i][2].isFloat())
            {
                step_height = preferences[i][2];
            }
            continue;
        }

        if (string(preferences[i][0]) == param_names[LOOP_TIME_LIMIT_MS])
        {
            if (preferences[i][2].isInt())
            {
                loop_time_limit_ms = preferences[i][2];
            }
            continue;
        }

        if (string(preferences[i][0]) == param_names[PREVIEW_SAMPLING_TIME_MS])
        {
            if (preferences[i][2].isInt())
            {
                preview_sampling_time_ms = preferences[i][2];
            }
            continue;
        }

        if (string(preferences[i][0]) == param_names[PREVIEW_WINDOW_SIZE])
        {
            if (preferences[i][2].isInt())
            {
                preview_window_size = preferences[i][2];
            }
            continue;
        }
    }
}


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
