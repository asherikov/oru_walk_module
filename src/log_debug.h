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

#define ORUW_TIMER(id,limit) oruw_timer timer(id,limit)
#define ORUW_TIMER_CHECK if(!timer.isLimitSatisfied())\
    {   stopWalking(); \
        throw ALERROR(getName(),__FUNCTION__,"Exec. time protection!");}
#else // ORUW_TIMER_ENABLE
#define ORUW_TIMER(id,limit)
#define ORUW_TIMER_CHECK
#endif // ORUW_TIMER_ENABLE


#ifdef ORUW_LOG_ENABLE
#include <cstdio>
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
        oruw_log (unsigned int);
        ~oruw_log ();

        void logJointValues (
                ALPtr<ALMemoryFastAccess>,
                ALPtr<ALMemoryFastAccess>);

        void logCoM (
                WMG *,
                nao_igm,
                ALPtr<ALMemoryFastAccess>);

        void logSwingFoot (
                nao_igm,
                ALPtr<ALMemoryFastAccess>);

        void logNumConstraints(const int);


    private:
        FILE *FJointsLog;
        FILE *FCoMLog;
        FILE *FSwingFootLog;
        avgFilter *com_filter;
        vector<float> sensorValues;
};

extern oruw_log *oruw_log_instance;

#define ORUW_LOG_OPEN(filter_len) oruw_log_instance = new oruw_log(filter_len)
#define ORUW_LOG_CLOSE delete oruw_log_instance
#define ORUW_LOG_JOINTS(sensors,actuators) oruw_log_instance->logJointValues(sensors,actuators)
#define ORUW_LOG_COM(wmg,nao,sensors) oruw_log_instance->logCoM(wmg,nao,sensors)
#define ORUW_LOG_SWING_FOOT(nao,sensors) oruw_log_instance->logSwingFoot(nao,sensors)
#define ORUW_LOG_NUM_CONSTRAINTS(num) oruw_log_instance->logNumConstraints(num)

#else // ORUW_LOG_ENABLE

#define ORUW_LOG_OPEN(filter_len) 
#define ORUW_LOG_CLOSE 
#define ORUW_LOG_JOINTS(sensors,actuators)
#define ORUW_LOG_COM(wmg,nao,sensors)
#define ORUW_LOG_SWING_FOOT(nao,sensors)
#define ORUW_LOG_NUM_CONSTRAINTS(num)

#endif // ORUW_LOG_ENABLE

#endif // LOG_DEBUG_H
