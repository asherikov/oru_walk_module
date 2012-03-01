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
        oruw_timer(const char*, const unsigned int);
        ~oruw_timer();

        bool isLimitSatisfied ();

    private:
        qi::os::timeval end_time;
        qi::os::timeval start_time;

        string id;
        double limit;
};

#define ORUW_TIMER(limit) oruw_timer timer(__FUNCTION__,limit)
#define ORUW_TIMER_CHECK if(!timer.isLimitSatisfied()) {halt("Time limit is violated!\n", __FUNCTION__);}

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

using namespace AL;
using namespace std;

class oruw_log
{
    public:
        oruw_log (const jointState&);
        ~oruw_log ();

        void logJointValues (const jointState&, const jointState&);

        void logCoM (smpc_parameters *, nao_igm&);

        void logFeet (nao_igm& nao);

        void logJointVelocities (const jointState&, const double);


        FILE *FJointsLog;
        FILE *FCoMLog;
        FILE *FFeetLog;
        FILE *FJointVelocities;
        FILE *FMessages;
        jointState state_old;
};

extern oruw_log *oruw_log_instance;

#define ORUW_LOG_IS_OPEN        (oruw_log_instance != NULL)
#define ORUW_LOG_OPEN(state)    oruw_log_instance = new oruw_log(state);

#define ORUW_LOG_CLOSE \
    if ORUW_LOG_IS_OPEN {delete oruw_log_instance; oruw_log_instance = NULL;}

#define ORUW_LOG_JOINTS(sensors,actuators) \
    if ORUW_LOG_IS_OPEN {oruw_log_instance->logJointValues(sensors,actuators);}

#define ORUW_LOG_COM(mpc,nao) \
    if ORUW_LOG_IS_OPEN {oruw_log_instance->logCoM(mpc,nao);}

#define ORUW_LOG_FEET(nao) \
    if ORUW_LOG_IS_OPEN {oruw_log_instance->logFeet(nao);}

#define ORUW_LOG_JOINT_VELOCITIES(current,time) \
    if ORUW_LOG_IS_OPEN {oruw_log_instance->logJointVelocities(current,time);}

#define ORUW_LOG_MESSAGE(...) \
    if ORUW_LOG_IS_OPEN {fprintf(oruw_log_instance->FMessages, __VA_ARGS__);}

#define ORUW_LOG_STEPS(wmg_pointer) \
    if (wmg_pointer != NULL) {wmg_pointer->FS2file("oru_steps_m.log", false);}

#else // ORUW_LOG_ENABLE

#define ORUW_LOG_OPEN(state) 
#define ORUW_LOG_CLOSE 
#define ORUW_LOG_JOINTS(sensors,actuators)
#define ORUW_LOG_COM(mpc,nao)
#define ORUW_LOG_FEET(nao)
#define ORUW_LOG_JOINT_VELOCITIES(current,time)
#define ORUW_LOG_MESSAGE(...)
#define ORUW_LOG_STEPS(wmg_pointer)

#endif // ORUW_LOG_ENABLE

#endif // LOG_DEBUG_H
