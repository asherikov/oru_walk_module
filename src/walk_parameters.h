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


enum walkPatterns
{
    WALK_PATTERN_STRAIGHT = 0,
    WALK_PATTERN_DIAGONAL = 1,
    WALK_PATTERN_CIRCULAR = 2
};


enum solverTypes
{
    SOLVER_TYPE_AS = 0,
    SOLVER_TYPE_IP = 1
};


enum parametersNames
{
    FEEDBACK_GAIN               ,
    FEEDBACK_THRESHOLD          ,

    MPC_SOLVER_TYPE             ,
    MPC_GAIN_POSITION           ,
    MPC_GAIN_VELOCITY           ,
    MPC_GAIN_ACCELERATION       ,
    MPC_GAIN_JERK               ,

    MPC_AS_TOLERANCE            ,
    MPC_AS_MAX_ACTIVATE         ,
    MPC_AS_USE_DOWNDATE         ,

    MPC_IP_TOLERANCE_INT        ,
    MPC_IP_TOLERANCE_EXT        ,
    MPC_IP_T                    ,
    MPC_IP_MU                   ,
    MPC_IP_BS_ALPHA             ,
    MPC_IP_BS_BETA              ,
    MPC_IP_MAX_ITER             ,
    MPC_IP_BS_TYPE              ,

    IGM_MU                      ,

    STEP_HEIGHT                 ,
    STEP_LENGTH                 ,
    BEZIER_WEIGHT_1             ,
    BEZIER_WEIGHT_2             ,
    BEZIER_INCLINATION_1        ,
    BEZIER_INCLINATION_2        ,

    LOOP_TIME_LIMIT_MS          ,
    DCM_TIME_SHIFT_MS           ,
    PREVIEW_SAMPLING_TIME_MS    ,
    PREVIEW_WINDOW_SIZE         ,
    SS_CONTROL_LOOPS            ,
    DS_CONTROL_LOOPS            ,
    DS_NUMBER                   ,

    STEP_PAIRS_NUMBER           ,
    WALK_PATTERN                ,

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

        int mpc_solver_type;

        double mpc_gain_position;
        double mpc_gain_velocity;
        double mpc_gain_acceleration;
        double mpc_gain_jerk;

        double mpc_as_tolerance;
        int mpc_as_max_activate;
        bool mpc_as_use_downdate;
        
        double mpc_ip_tolerance_int;
        double mpc_ip_tolerance_ext;
        double mpc_ip_t;
        double mpc_ip_mu;
        double mpc_ip_bs_alpha;
        double mpc_ip_bs_beta;
        int mpc_ip_max_iter;
        int mpc_ip_bs_type;


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

        int walk_pattern;


        double igm_tol;
        int igm_max_iter;
        double igm_mu;


        double bezier_weight_1;
        double bezier_weight_2;
        double bezier_inclination_1;
        double bezier_inclination_2;


        ALValue param_names;
        ALPreferencesProxy pref_proxy;
};

#endif  // WALK_PARAMETERS_H
