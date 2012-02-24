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
        ORUW_LOG_MESSAGE("Timer '%s' (sec): %f > %f (limit)\n", id.c_str(), timediff, limit);
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

    ORUW_LOG_MESSAGE("Timer '%s' (sec): %f\n", id.c_str(), timediff);
}
#endif // ORUW_TIMER_ENABLE




#ifdef ORUW_LOG_ENABLE
oruw_log *oruw_log_instance = NULL;


oruw_log::oruw_log (const jointState& state_init)
{
    state_old = state_init;
    FJointsLog = fopen ("./oru_joints.log", "w");
    FCoMLog = fopen ("./oru_com.log", "w");
    FFeetLog = fopen ("./oru_feet.log", "w");
    FJointVelocities = fopen ("./oru_joint_velocities.log", "w");
    FMessages = fopen ("./oru_messages.log", "w");
}

oruw_log::~oruw_log ()
{
    fclose (FJointsLog);
    fclose (FCoMLog);
    fclose (FFeetLog);
    fclose (FJointVelocities);
    fclose (FMessages);
}



void oruw_log::logJointValues(
        const jointState& state_sensor,
        const jointState& state_expected)
{
    for (int i = 0; i < JOINTS_NUM; i++)
    {
        fprintf (FJointsLog, "%f ", state_sensor.q[i]);
    }
    fprintf (FJointsLog, "    ");


    for (int i = 0; i < JOINTS_NUM; i++)
    {
        fprintf (FJointsLog, "%f ", state_expected.q[i]);
    }
    fprintf (FJointsLog, "\n");
}



void oruw_log::logCoM(
        smpc_parameters *mpc,
        nao_igm& nao)
{
    fprintf (FCoMLog, "%f %f %f    ", mpc->init_state.x(), mpc->init_state.y(), mpc->hCoM);


    double CoM[3];
    nao.getCoM(nao.state_sensor, CoM);

    fprintf (FCoMLog, "%f %f %f\n", CoM[0], CoM[1], CoM[2]);
}


void oruw_log::logFeet(nao_igm& nao)
{
    double l_expected[POSITION_VECTOR_SIZE];
    double l_real[POSITION_VECTOR_SIZE];
    double r_expected[POSITION_VECTOR_SIZE];
    double r_real[POSITION_VECTOR_SIZE];

    nao.getFeetPositions (
            l_expected,
            r_expected,
            l_real,
            r_real);

    fprintf (FFeetLog, "%f %f %f    %f %f %f", 
            l_expected[0], l_expected[1], l_expected[2], 
            l_real[0], l_real[1], l_real[2]);
    fprintf (FFeetLog, "     %f %f %f    %f %f %f\n", 
            r_expected[0], r_expected[1], r_expected[2],
            r_real[0], r_real[1], r_real[2]);
}


void oruw_log::logJointVelocities (const jointState& state_current, const double time)
{
    for (int i = 0; i < JOINTS_NUM; i++)
    {
        fprintf (FJointVelocities, "%f ", fabs(state_old.q[i] - state_current.q[i]) / time);
        state_old.q[i] = state_current.q[i];
    }
    fprintf (FJointVelocities, "\n");
}
#endif // ORUW_LOG_ENABLE
