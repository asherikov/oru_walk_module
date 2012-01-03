/**
 * @file
 * @author Alexander Sherikov
 * @date 03.01.2012 19:17:44 MSK
 */


#include "log_debug.h"


#ifdef ORUW_TIMER_ENABLE
oruw_timer::oruw_timer()
{
    qi::os::gettimeofday (&start_time);
}

oruw_timer::~oruw_timer()
{
    qi::os::gettimeofday (&end_time);
    qiLogInfo ("module.oru_walk", "Exec time (sec): %f\n",
        (double) end_time.tv_sec - start_time.tv_sec + 0.000001* (end_time.tv_usec - start_time.tv_usec));
}
#endif // ORUW_TIMER_ENABLE


#ifdef ORUW_LOG_ENABLE

oruw_log::oruw_log ()
{
    FJointsLog = fopen ("./oru_joints.log", "w");
    FCoMLog = fopen ("./oru_com.log", "w");
}
oruw_log::~oruw_log ()
{
    fclose (FJointsLog);
    fclose (FCoMLog);
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

oruw_log *oruw_log_instance;
#endif // ORUW_LOG_ENABLE
