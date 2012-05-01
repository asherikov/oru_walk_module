/**
 * @file
 * @author Alexander Sherikov
 * @date 03.01.2012 19:17:44 MSK
 */


#ifndef ORUW_LOG_H
#define ORUW_LOG_H

/**
 * Enable logging.
 */
#define ORUW_LOG_ENABLE


#ifdef ORUW_LOG_ENABLE
#include <cstdio>
#include <qi/log.hpp>

#include "nao_igm.h"
#include "smpc_solver.h"
#include "WMG.h"
#include "joints_sensors_id.h"
#include "walk_parameters.h"


class oruw_log
{
    public:
        oruw_log ();
        ~oruw_log ();

        void logJointValues (const jointState&, const jointState&);
        void logCoM (smpc_parameters&, nao_igm&);
        void logFeet (nao_igm& nao);
        void logSolverInfo (smpc::solver *, int);


        FILE *FJointsLog;
        FILE *FCoMLog;
        FILE *FFeetLog;
        FILE *FMessages;
};


extern oruw_log *oruw_log_instance;


#define ORUW_LOG_IS_OPEN    (oruw_log_instance != NULL)
#define ORUW_LOG_OPEN       oruw_log_instance = new oruw_log();

#define ORUW_LOG_CLOSE \
    if ORUW_LOG_IS_OPEN {delete oruw_log_instance; oruw_log_instance = NULL;}

#define ORUW_LOG_JOINTS(sensors,actuators) \
    if ORUW_LOG_IS_OPEN {oruw_log_instance->logJointValues(sensors,actuators);}

#define ORUW_LOG_COM(mpc,nao) \
    if ORUW_LOG_IS_OPEN {oruw_log_instance->logCoM(mpc,nao);}

#define ORUW_LOG_FEET(nao) \
    if ORUW_LOG_IS_OPEN {oruw_log_instance->logFeet(nao);}

#define ORUW_LOG_MESSAGE(...) \
    if ORUW_LOG_IS_OPEN {fprintf(oruw_log_instance->FMessages, __VA_ARGS__);}

#define ORUW_LOG_SOLVER_INFO \
    if ORUW_LOG_IS_OPEN {oruw_log_instance->logSolverInfo(solver, wp.mpc_solver_type);}

#define ORUW_LOG_STEPS(wmg) \
    if ORUW_LOG_IS_OPEN {wmg.FS2file("oru_steps_m.log", false);}


#else // ORUW_LOG_ENABLE


#define ORUW_LOG_OPEN 
#define ORUW_LOG_CLOSE 
#define ORUW_LOG_JOINTS(sensors,actuators)
#define ORUW_LOG_COM(mpc,nao)
#define ORUW_LOG_FEET(nao)
#define ORUW_LOG_MESSAGE(...)
#define ORUW_LOG_STEPS(wmg)
#define ORUW_LOG_SOLVER_INFO


#endif // ORUW_LOG_ENABLE

#endif // ORUW_LOG_H
