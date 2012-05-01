/**
 * @file
 * @author Antonio Paolillo
 * @author Dimitar Dimitrov
 * @author Alexander Sherikov
 */

#include "oru_walk.h"
#include "oruw_log.h"
#include "oruw_timer.h"


/**
 * @brief Initialize selected walk pattern
 */
void oru_walk::initWalkPattern(WMG &wmg)
{
    // support foot position and orientation
    nao.init (
            IGM_SUPPORT_LEFT,
            0.0, 0.05, 0.0, // position
            0.0, 0.0, 0.0);  // orientation
    // swing foot position
    nao.getSwingFootPosture (nao.state_sensor, nao.right_foot_posture.data());


    switch (wp.walk_pattern)
    {
        case WALK_PATTERN_STRAIGHT:
            initWalkPattern_Straight(wmg);
            break;
        case WALK_PATTERN_DIAGONAL:
            initWalkPattern_Diagonal(wmg);
            break;
        case WALK_PATTERN_CIRCULAR:
            initWalkPattern_Circular(wmg);
            break;
        default:
            halt("Unknown walk pattern.\n", __FUNCTION__);
            break;
    }
}


/**
 * @brief Initializes walk pattern
 */
void oru_walk::initWalkPattern_Straight(WMG &wmg)
{
    // each step is defined relatively to the previous step
    const double step_x = wp.step_length;                        // relative X position
    const double step_y = wmg.def_constraints.support_distance_y;// relative Y position


    wmg.setFootstepParametersMS(0, 0, 0);
    wmg.addFootstep(0.0, step_y/2, 0.0, FS_TYPE_SS_L);

    // Initial double support
    wmg.setFootstepParametersMS(3*wp.ss_time_ms, 0, 0);
    wmg.addFootstep(0.0, -step_y/2, 0.0, FS_TYPE_DS);


    // all subsequent steps have normal feet size
    wmg.setFootstepParametersMS(wp.ss_time_ms, 0, 0);
    wmg.addFootstep(0.0   , -step_y/2, 0.0);
    wmg.setFootstepParametersMS(wp.ss_time_ms, wp.ds_time_ms, wp.ds_number);
    wmg.addFootstep(step_x,  step_y,   0.0);

    for (int i = 0; i < wp.step_pairs_number; i++)
    {
        wmg.addFootstep(step_x, -step_y, 0.0);
        wmg.addFootstep(step_x,  step_y, 0.0);
    }

    // here we give many reference points, since otherwise we 
    // would not have enough steps in preview window to reach 
    // the last footsteps
    wmg.setFootstepParametersMS(5*wp.ss_time_ms, 0, 0);
    wmg.addFootstep(0.0   , -step_y/2, 0.0, FS_TYPE_DS);
    wmg.setFootstepParametersMS(0, 0, 0);
    wmg.addFootstep(0.0   , -step_y/2, 0.0, FS_TYPE_SS_R);
}



/**
 * @brief Initializes walk pattern
 */
void oru_walk::initWalkPattern_Diagonal(WMG &wmg)
{
    // each step is defined relatively to the previous step
    const double step_x = wp.step_length;                        // relative X position
    const double step_y = wmg.def_constraints.support_distance_y;// relative Y position


    wmg.setFootstepParametersMS (0, 0, 0);
    wmg.addFootstep(0.0, step_y/2, 0.0, FS_TYPE_SS_L);

    // Initial double support
    wmg.setFootstepParametersMS (3*wp.ss_time_ms, 0, 0);
    wmg.addFootstep(0.0, -step_y/2, 0.0, FS_TYPE_DS);

    // each step is defined relatively to the previous step
    const double shift = -0.01;

    // all subsequent steps have normal feet size
    wmg.setFootstepParametersMS (wp.ss_time_ms, wp.ds_time_ms, wp.ds_number);
    wmg.addFootstep(0.0   , -step_y/2, 0.0);
    wmg.addFootstep(step_x,  step_y + shift, 0.0);

    for (int i = 0; i < wp.step_pairs_number; i++)
    {
        wmg.addFootstep(step_x, -step_y + shift, 0.0);
        wmg.addFootstep(step_x,  step_y + shift, 0.0);
    }


    // here we give many reference points, since otherwise we 
    // would not have enough steps in preview window to reach 
    // the last footsteps
    wmg.setFootstepParametersMS (6*wp.ss_time_ms, 0, 0);
    wmg.addFootstep(0.0   , -step_y/2, 0.0, FS_TYPE_DS);
    wmg.setFootstepParametersMS (0, 0, 0);
    wmg.addFootstep(0.0   , -step_y/2, 0.0, FS_TYPE_SS_R);
}




/**
 * @brief Initializes walk pattern
 */
void oru_walk::initWalkPattern_Circular(WMG &wmg)
{
    // each step is defined relatively to the previous step
    const double step_x_ext = wp.step_length;                    // relative X position
    const double step_y = wmg.def_constraints.support_distance_y;// relative Y position

    const double R_ext = 0.55;
    const double R_int = R_ext - step_y;

    // relative angle
    double a = asin (step_x_ext / R_ext);
    double step_x_int = step_x_ext * R_int / R_ext;


    wmg.setFootstepParametersMS (0, 0, 0);
    wmg.addFootstep(0.0, step_y/2, 0.0, FS_TYPE_SS_L);

    // Initial double support
    wmg.setFootstepParametersMS (3*wp.ss_time_ms, 0, 0);
    wmg.addFootstep(0.0, -step_y/2, 0.0, FS_TYPE_DS);



    wmg.setFootstepParametersMS (wp.ss_time_ms, wp.ds_time_ms, wp.ds_number);
    wmg.addFootstep(0.0   ,     -step_y/2, 0.0);
    wmg.addFootstep(step_x_int,  step_y, a);

    for (int i = 0; i < wp.step_pairs_number; i++)
    {
        wmg.addFootstep(step_x_ext, -step_y, a);
        wmg.addFootstep(step_x_int,  step_y, a);
    }

    // here we give many reference points, since otherwise we 
    // would not have enough steps in preview window to reach 
    // the last footsteps
    wmg.setFootstepParametersMS (6*wp.ss_time_ms, 0, 0);
    wmg.addFootstep(0.0   , -step_y/2, 0.0, FS_TYPE_DS);
    wmg.setFootstepParametersMS (0, 0, 0);
    wmg.addFootstep(0.0   , -step_y/2, 0.0, FS_TYPE_SS_R);
}
