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
#include <string>

#include <qi/os.hpp>
#include <qi/log.hpp>
#include <alcore/alerror.h>


using namespace std;


/**
 * @brief Log time of existance on destruction. Throw an error, if the given
 * upper limit is not satisfied.
 */
class oruw_timer
{
    public:
        oruw_timer(const char* timer_id, const unsigned int timer_limit);
        ~oruw_timer();

        bool isLimitSatisfied ();

    private:
        qi::os::timeval end_time;
        qi::os::timeval start_time;

        string id;
        double limit;
};

#define ORUW_TIMER(limit) oruw_timer timer(__FUNCTION__,limit)
#define ORUW_TIMER_CHECK if(!timer.isLimitSatisfied()){ORUW_HALT("Timer protection!\n");}

#else // ORUW_TIMER_ENABLE

#define ORUW_TIMER(limit)
#define ORUW_TIMER_CHECK

#endif // ORUW_TIMER_ENABLE


#ifdef ORUW_LOG_ENABLE
#include <cstdio>
#include <cmath>
#include <alcore/alptr.h>
#include <almemoryfastaccess/almemoryfastaccess.h>
#include <qi/log.hpp>

#include "nao_igm.h"
#include "WMG.h"
#include "joints_sensors_id.h"
#include "avg_filter.h"

using namespace AL;
using namespace std;

class oruw_log
{
    public:
        oruw_log (const modelState&, const unsigned int);
        ~oruw_log ();

        void logJointValues (const modelState&, const modelState&);

        void logCoM (WMG *, modelState&);

        void logSwingFoot (modelState&, modelState&);

        void logNumConstraints(const int);

        void logJointVelocities (const modelState&, const double);


    private:
        FILE *FJointsLog;
        FILE *FCoMLog;
        FILE *FSwingFootLog;
        FILE *FJointVelocities;
        avgFilter *com_filter;
        modelState state_old;
};

extern oruw_log *oruw_log_instance;

#define ORUW_LOG_OPEN(state,filter_len) oruw_log_instance = new oruw_log(state,filter_len)
#define ORUW_LOG_CLOSE delete oruw_log_instance
#define ORUW_LOG_JOINTS(sensors,actuators) oruw_log_instance->logJointValues(sensors,actuators)
#define ORUW_LOG_COM(wmg,sensors) oruw_log_instance->logCoM(wmg,sensors)
#define ORUW_LOG_SWING_FOOT(sensors,actuators) oruw_log_instance->logSwingFoot(sensors,actuators)
#define ORUW_LOG_NUM_CONSTRAINTS(num) oruw_log_instance->logNumConstraints(num)
#define ORUW_LOG_JOINT_VELOCITIES(current,time) oruw_log_instance->logJointVelocities(current,time);

#else // ORUW_LOG_ENABLE

#define ORUW_LOG_OPEN(state,filter_len) 
#define ORUW_LOG_CLOSE 
#define ORUW_LOG_JOINTS(sensors,actuators)
#define ORUW_LOG_COM(wmg,sensors)
#define ORUW_LOG_SWING_FOOT(sensors,actuators)
#define ORUW_LOG_NUM_CONSTRAINTS(num)
#define ORUW_LOG_JOINT_VELOCITIES(current,time)

#endif // ORUW_LOG_ENABLE

#endif // LOG_DEBUG_H
