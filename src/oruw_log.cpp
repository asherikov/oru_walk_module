/**
 * @file
 * @author Alexander Sherikov
 * @date 03.01.2012 19:17:44 MSK
 */


#include "oruw_log.h"


#ifdef ORUW_LOG_ENABLE
oruw_log *oruw_log_instance = NULL;


oruw_log::oruw_log ()
{
    FJointsLog = fopen ("./oru_joints.log", "w");
    FCoMLog = fopen ("./oru_com.log", "w");
    FFeetLog = fopen ("./oru_feet.log", "w");
    FMessages = fopen ("./oru_messages.log", "w");
}


oruw_log::~oruw_log ()
{
    fclose (FJointsLog);
    fclose (FCoMLog);
    fclose (FFeetLog);
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
        smpc_parameters &mpc,
        nao_igm& nao)
{
    fprintf (FCoMLog, "%f %f %f    ", mpc.init_state.x(), mpc.init_state.y(), mpc.hCoM);


    double CoM[POSITION_VECTOR_SIZE];
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
#endif // ORUW_LOG_ENABLE
