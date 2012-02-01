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
#include <vector>


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
    LOOP_TIME_LIMIT_MS          = 9,
    PREVIEW_SAMPLING_TIME_MS    = 10,
    PREVIEW_WINDOW_SIZE         = 11,

    NUM_PARAMETERS              = 12
};


/**
 * @brief A container for constant parameters.
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

        int control_sampling_time_ms;
        int loop_time_limit_ms;
        int preview_sampling_time_ms;
        int preview_window_size;

        unsigned int filter_window_length;


        vector<string> param_names;
        ALPreferencesProxy pref_proxy;
};

#endif  // WALK_PARAMETERS_H
