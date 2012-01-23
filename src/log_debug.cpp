/**
 * @file
 * @author Alexander Sherikov
 * @date 03.01.2012 19:17:44 MSK
 */


#include "log_debug.h"


#ifdef ORUW_TIMER_ENABLE
/**
 * @brief Create timer
 *
 * @param[in] timer_id identifier of the timer
 * @param[in] timer_limit the upper limit
 */
oruw_timer::oruw_timer(const char* timer_id, const unsigned int timer_limit) : id(timer_id)
{
    qi::os::gettimeofday (&start_time);
    limit = (double) timer_limit / 1000000;
}


bool oruw_timer::isLimitSatisfied ()
{
    qi::os::gettimeofday (&end_time);
    double timediff = (double) end_time.tv_sec - start_time.tv_sec + 0.000001* (end_time.tv_usec - start_time.tv_usec);
    if (timediff > limit)
    {
        qiLogInfo ("module.oru_walk", "Timer '%s' (sec): %f > %f (limit)\n", id.c_str(), timediff, limit);
        return (false);
    }

    return (true);
}


/**
 * @brief Destroy the timer: log time, throw an error if needed.
 */
oruw_timer::~oruw_timer()
{
    qi::os::gettimeofday (&end_time);
    double timediff = (double) end_time.tv_sec - start_time.tv_sec + 0.000001* (end_time.tv_usec - start_time.tv_usec);

    qiLogInfo ("module.oru_walk", "Timer '%s' (sec): %f\n", id.c_str(), timediff);
}
#endif // ORUW_TIMER_ENABLE




#ifdef ORUW_LOG_ENABLE
oruw_log *oruw_log_instance;


oruw_log::oruw_log (unsigned int filter_window_len)
{
    FJointsLog = fopen ("./oru_joints.log", "w");
    FCoMLog = fopen ("./oru_com.log", "w");
    FSwingFootLog = fopen ("./oru_swing_foot.log", "w");
    com_filter = new avgFilter(filter_window_len);
}

oruw_log::~oruw_log ()
{
    fclose (FJointsLog);
    fclose (FCoMLog);
    fclose (FSwingFootLog);
    delete com_filter;
}



void oruw_log::logJointValues(
        const modelState& state_sensor,
        const modelState& state_expected)
{
    fprintf (FJointsLog, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f    ",
                        state_sensor.q[HEAD_PITCH],
                        state_sensor.q[HEAD_YAW],
                        //
                        state_sensor.q[L_ANKLE_PITCH],
                        state_sensor.q[L_ANKLE_ROLL],
                        state_sensor.q[L_ELBOW_ROLL],
                        state_sensor.q[L_ELBOW_YAW],
                        state_sensor.q[L_HIP_PITCH],
                        state_sensor.q[L_HIP_ROLL],
                        state_sensor.q[L_HIP_YAW_PITCH],
                        state_sensor.q[L_KNEE_PITCH],
                        state_sensor.q[L_SHOULDER_PITCH],
                        state_sensor.q[L_SHOULDER_ROLL],
                        state_sensor.q[L_WRIST_YAW],
                        //
                        state_sensor.q[R_ANKLE_PITCH],
                        state_sensor.q[R_ANKLE_ROLL],
                        state_sensor.q[R_ELBOW_ROLL],
                        state_sensor.q[R_ELBOW_YAW],
                        state_sensor.q[R_HIP_PITCH],
                        state_sensor.q[R_HIP_ROLL],
                        state_sensor.q[R_KNEE_PITCH],
                        state_sensor.q[R_SHOULDER_PITCH],
                        state_sensor.q[R_SHOULDER_ROLL],
                        state_sensor.q[R_WRIST_YAW]);

    fprintf (FJointsLog, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
                        state_expected.q[HEAD_PITCH],
                        state_expected.q[HEAD_YAW],
                        //
                        state_expected.q[L_ANKLE_PITCH],
                        state_expected.q[L_ANKLE_ROLL],
                        state_expected.q[L_ELBOW_ROLL],
                        state_expected.q[L_ELBOW_YAW],
                        state_expected.q[L_HIP_PITCH],
                        state_expected.q[L_HIP_ROLL],
                        state_expected.q[L_HIP_YAW_PITCH],
                        state_expected.q[L_KNEE_PITCH],
                        state_expected.q[L_SHOULDER_PITCH],
                        state_expected.q[L_SHOULDER_ROLL],
                        state_expected.q[L_WRIST_YAW],
                        //
                        state_expected.q[R_ANKLE_PITCH],
                        state_expected.q[R_ANKLE_ROLL],
                        state_expected.q[R_ELBOW_ROLL],
                        state_expected.q[R_ELBOW_YAW],
                        state_expected.q[R_HIP_PITCH],
                        state_expected.q[R_HIP_ROLL],
                        state_expected.q[R_KNEE_PITCH],
                        state_expected.q[R_SHOULDER_PITCH],
                        state_expected.q[R_SHOULDER_ROLL],
                        state_expected.q[R_WRIST_YAW]);
}



void oruw_log::logCoM(
        WMG *wmg,
        modelState& state_sensor)
{
    fprintf (FCoMLog, "%f %f %f    ", wmg->init_state.x(), wmg->init_state.y(), wmg->hCoM);


    double CoM[3];
    state_sensor.getCoM(CoM);
    //com_filter->addValue(CoM[0], CoM[1], CoM[0], CoM[1]);

    fprintf (FCoMLog, "%f %f %f\n", CoM[0], CoM[1], CoM[2]);
}


void oruw_log::logSwingFoot(
        modelState& state_sensor,
        modelState& state_expected)
{
    double swing_foot[3];


    state_expected.getSwingFoot(swing_foot);
    fprintf (FSwingFootLog, "%f %f %f    ", swing_foot[0], swing_foot[1], swing_foot[2]);

    state_sensor.getSwingFoot(swing_foot);
    fprintf (FSwingFootLog, "%f %f %f\n", swing_foot[0], swing_foot[1], swing_foot[2]);
}


void oruw_log::logNumConstraints(const int num)
{
    qiLogInfo ("module.oru_walk", "Num of active constraints: %d\n", num);
}
#endif // ORUW_LOG_ENABLE
