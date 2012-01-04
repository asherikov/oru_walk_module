/**
 * @file
 * @author Alexander Sherikov
 * @date 03.01.2012 19:17:44 MSK
 */


#ifndef LOG_DEBUG_H
#define LOG_DEBUG_H

/**
 * Measure execution time of the callback function.
 * By default naoqi prints logs to stdout, when executed on a PC,
 * the location of the log file on the robot is /var/log/naoqi.log
 */
#define ORUW_TIMER_ENABLE

/**
 * Enable logging.
 */
#define ORUW_LOG_ENABLE




#ifdef ORUW_TIMER_ENABLE
#include <qi/os.hpp>
#include <qi/log.hpp>

/**
 * @brief Log time of existance on destruction.
 */
class oruw_timer
{
    public:
        oruw_timer();
        ~oruw_timer();

    private:
        qi::os::timeval end_time;
        qi::os::timeval start_time;
};

#define ORUW_TIMER oruw_timer timer
#else // ORUW_TIMER_ENABLE
#define ORUW_TIMER
#endif // ORUW_TIMER_ENABLE


#ifdef ORUW_LOG_ENABLE
#include <cstdio>
#include <alcore/alptr.h>
#include <almemoryfastaccess/almemoryfastaccess.h>

#include "nao_igm.h"
#include "joints_sensors_id.h"

using namespace AL;
using namespace std;

class oruw_log
{
    public:
        oruw_log ();
        ~oruw_log ();

        void logJointValues (
                ALPtr<ALMemoryFastAccess> accessSensorValues,
                ALPtr<ALMemoryFastAccess> accessActuatorValues);

        void logCoM (
                nao_igm nao,
                ALPtr<ALMemoryFastAccess> accessSensorValues);

        void logSwingFoot (
                nao_igm nao,
                ALPtr<ALMemoryFastAccess> accessSensorValues);

    private:
        FILE *FJointsLog;
        FILE *FCoMLog;
        FILE *FSwingFootLog;
        vector<float> sensorValues;
};

extern oruw_log *oruw_log_instance;

#define ORUW_LOG_OPEN oruw_log_instance = new oruw_log
#define ORUW_LOG_CLOSE delete oruw_log_instance
#define ORUW_LOG_JOINTS(sensors,actuators) oruw_log_instance->logJointValues(sensors,actuators)
#define ORUW_LOG_COM(nao,sensors) oruw_log_instance->logCoM(nao,sensors)
#define ORUW_LOG_SWING_FOOT(nao,sensors) oruw_log_instance->logSwingFoot(nao,sensors)

#else // ORUW_LOG_ENABLE

#define ORUW_LOG_OPEN 
#define ORUW_LOG_CLOSE 
#define ORUW_LOG_JOINTS(sensors,actuators)
#define ORUW_LOG_COM(nao,sensors)
#define ORUW_LOG_SWING_FOOT(nao,sensors)

#endif // ORUW_LOG_ENABLE

#endif // LOG_DEBUG_H
