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
    FEEDBACK_GAIN               ,
    FEEDBACK_THRESHOLD          ,
    MPC_ALPHA                   ,
    MPC_BETA                    ,
    MPC_GAMMA                   ,
    MPC_REGULARIZATION          ,
    MPC_TOLERANCE               ,
    STEP_HEIGHT                 ,
    STEP_LENGTH                 ,
    LOOP_TIME_LIMIT_MS          ,
    DCM_TIME_SHIFT_MS           ,
    PREVIEW_SAMPLING_TIME_MS    ,
    PREVIEW_WINDOW_SIZE         ,
    SS_CONTROL_LOOPS            ,
    DS_CONTROL_LOOPS            ,
    DS_NUMBER                   ,
    STEP_PAIRS_NUMBER           ,

    NUM_PARAMETERS              
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

        double mpc_alpha;
        double mpc_beta;
        double mpc_gamma;
        double mpc_regularization;
        double mpc_tolerance;

        double step_height;
        double step_length;

        int walk_control_thread_priority;

        int dcm_sampling_time_ms;
        int dcm_time_shift_ms;
        int control_sampling_time_ms;
        double control_sampling_time_sec;
        int loop_time_limit_ms;
        int preview_sampling_time_ms;
        double preview_sampling_time_sec;
        int preview_window_size;

        bool set_support_z_to_zero;

        int ss_time_ms;
        int ds_time_ms;
        int ds_number;
        int step_pairs_number;

        ALValue param_names;
        ALPreferencesProxy pref_proxy;
};

#endif  // WALK_PARAMETERS_H
