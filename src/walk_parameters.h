/**
 * @file
 * @author Antonio Paolillo
 * @author Dimitar Dimitrov
 * @author Alexander Sherikov
 */


#ifndef WALK_PARAMETERS_H
#define WALK_PARAMETERS_H


//----------------------------------------
// INCLUDES
//----------------------------------------

#include <string>


#include <qi/log.hpp>
#include <alcore/alptr.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <alproxies/alpreferencesproxy.h>




//----------------------------------------
// DEFINITIONS
//----------------------------------------


using namespace AL;
using namespace std;

enum parametersNames
{
    FEEDBACK_GAIN               = 0,
    FEEDBACK_THRESHOLD          = 1,
    JOINT_FEEDBACK_GAIN         = 2,
    MPC_ALPHA                   = 3,
    MPC_BETA                    = 4,
    MPC_GAMMA                   = 5,
    MPC_REGULARIZATION          = 6,
    MPC_TOLERANCE               = 7,
    STEP_HEIGHT                 = 8,
    STEP_LENGTH                 = 9,
    LOOP_TIME_LIMIT_MS          = 10,
    PREVIEW_SAMPLING_TIME_MS    = 11,
    PREVIEW_WINDOW_SIZE         = 12,
    SS_NUMBER                   = 13,
    DS_NUMBER                   = 14,
    STEP_PAIRS_NUMBER           = 15,

    NUM_PARAMETERS              = 16
};


/**
 * @brief A container for parameters.
 */
class walkParameters
{
    public:
        walkParameters(ALPtr<ALBroker>);
        void readParameters();
        void writeParameters();


        double feedback_gain;
        double feedback_threshold;

        double joint_feedback_gain;

        double mpc_alpha;
        double mpc_beta;
        double mpc_gamma;
        double mpc_regularization;
        double mpc_tolerance;

        double step_height;
        double step_length;

        int control_sampling_time_ms;
        double control_sampling_time_sec;
        int loop_time_limit_ms;
        int preview_sampling_time_ms;
        double preview_sampling_time_sec;
        int preview_window_size;

        unsigned int filter_window_length;

        bool set_support_z_to_zero;

        int ss_number;
        int ds_number;
        int step_pairs_number;

        ALValue param_names;
        ALPreferencesProxy pref_proxy;
};

#endif  // WALK_PARAMETERS_H
