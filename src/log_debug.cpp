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

/**
 * @brief Destroy the timer: log time, throw an error if needed.
 */
oruw_timer::~oruw_timer()
{
    qi::os::gettimeofday (&end_time);
    double timediff = (double) end_time.tv_sec - start_time.tv_sec + 0.000001* (end_time.tv_usec - start_time.tv_usec);

    qiLogInfo ("module.oru_walk", "Timer '%s' (sec): %f\n", id.c_str(), timediff);
    if (timediff > limit)
    {
        throw ALERROR("oru_walk", __FUNCTION__, "Time limit is not satisfied!");
    }
}
#endif // ORUW_TIMER_ENABLE



#ifdef ORUW_LOG_ENABLE
oruw_log *oruw_log_instance;


oruw_log::oruw_log ()
{
    FJointsLog = fopen ("./oru_joints.log", "w");
    FCoMLog = fopen ("./oru_com.log", "w");
    FSwingFootLog = fopen ("./oru_swing_foot.log", "w");
    com_filter = new avgFilter(10);
}

oruw_log::~oruw_log ()
{
    fclose (FJointsLog);
    fclose (FCoMLog);
    fclose (FSwingFootLog);
    delete com_filter;
}



void oruw_log::logJointValues(
        ALPtr<ALMemoryFastAccess> accessSensorValues,
        ALPtr<ALMemoryFastAccess> accessActuatorValues)
{
    accessSensorValues->GetValues (sensorValues);
    fprintf (FJointsLog, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f    ",
                        sensorValues[HEAD_PITCH],
                        sensorValues[HEAD_YAW],
                        //
                        sensorValues[L_ANKLE_PITCH],
                        sensorValues[L_ANKLE_ROLL],
                        sensorValues[L_ELBOW_ROLL],
                        sensorValues[L_ELBOW_YAW],
                        sensorValues[L_HIP_PITCH],
                        sensorValues[L_HIP_ROLL],
                        sensorValues[L_HIP_YAW_PITCH],
                        sensorValues[L_KNEE_PITCH],
                        sensorValues[L_SHOULDER_PITCH],
                        sensorValues[L_SHOULDER_ROLL],
                        sensorValues[L_WRIST_YAW],
                        //
                        sensorValues[R_ANKLE_PITCH],
                        sensorValues[R_ANKLE_ROLL],
                        sensorValues[R_ELBOW_ROLL],
                        sensorValues[R_ELBOW_YAW],
                        sensorValues[R_HIP_PITCH],
                        sensorValues[R_HIP_ROLL],
                        sensorValues[R_KNEE_PITCH],
                        sensorValues[R_SHOULDER_PITCH],
                        sensorValues[R_SHOULDER_ROLL],
                        sensorValues[R_WRIST_YAW]);

    accessActuatorValues->GetValues (sensorValues);
    fprintf (FJointsLog, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
                        sensorValues[HEAD_PITCH],
                        sensorValues[HEAD_YAW],
                        //
                        sensorValues[L_ANKLE_PITCH],
                        sensorValues[L_ANKLE_ROLL],
                        sensorValues[L_ELBOW_ROLL],
                        sensorValues[L_ELBOW_YAW],
                        sensorValues[L_HIP_PITCH],
                        sensorValues[L_HIP_ROLL],
                        sensorValues[L_HIP_YAW_PITCH],
                        sensorValues[L_KNEE_PITCH],
                        sensorValues[L_SHOULDER_PITCH],
                        sensorValues[L_SHOULDER_ROLL],
                        sensorValues[L_WRIST_YAW],
                        //
                        sensorValues[R_ANKLE_PITCH],
                        sensorValues[R_ANKLE_ROLL],
                        sensorValues[R_ELBOW_ROLL],
                        sensorValues[R_ELBOW_YAW],
                        sensorValues[R_HIP_PITCH],
                        sensorValues[R_HIP_ROLL],
                        sensorValues[R_KNEE_PITCH],
                        sensorValues[R_SHOULDER_PITCH],
                        sensorValues[R_SHOULDER_ROLL],
                        sensorValues[R_WRIST_YAW]);
}



void oruw_log::logCoM(
        WMG *wmg,
        nao_igm nao,
        ALPtr<ALMemoryFastAccess> accessSensorValues)
{
    fprintf (FCoMLog, "%f %f %f    ", wmg->init_state.x(), wmg->init_state.y(), wmg->hCoM);


    accessSensorValues->GetValues (sensorValues);
    for (int i = 0; i < JOINTS_NUM; i++)
    {
        nao.q[i] = sensorValues[i];
    }
    double CoM[3];
    nao.getUpdatedCoM(CoM);
    //com_filter->addValue(CoM[0], CoM[1], CoM[0], CoM[1]);

    fprintf (FCoMLog, "%f %f %f\n", CoM[0], CoM[1], CoM[2]);
}


void oruw_log::logSwingFoot(
        nao_igm nao,
        ALPtr<ALMemoryFastAccess> accessSensorValues)
{
    double swing_foot[3];


    nao.getUpdatedSwingFoot(swing_foot);
    fprintf (FSwingFootLog, "%f %f %f    ", swing_foot[0], swing_foot[1], swing_foot[2]);


    accessSensorValues->GetValues (sensorValues);
    for (int i = 0; i < JOINTS_NUM; i++)
    {
        nao.q[i] = sensorValues[i];
    }
    nao.getUpdatedSwingFoot(swing_foot);
    fprintf (FSwingFootLog, "%f %f %f\n", swing_foot[0], swing_foot[1], swing_foot[2]);
}
#endif // ORUW_LOG_ENABLE
