/**
 * @brief Walk straight
 */
void init_04 (WMG *wmg)
{
    double d[4];
    wmg->init(15);

    // each step is defined relatively to the previous step
    double step_x = 0.035;      // relative X position
    double step_y = 0.1;       // relative Y position


    d[0] = 0.09;
    d[1] = 0.025;
    d[2] = 0.03;
    d[3] = 0.025;
    wmg->AddFootstep(0.0, step_y/2, 0.0, 0, 0, d, FS_TYPE_SS_L);

    // Initial double support
    d[0] = 0.09;
    d[1] = 0.075;
    d[2] = 0.03;
    d[3] = 0.075;
    wmg->AddFootstep(0.0, -step_y/2, 0.0, 2, 2, d, FS_TYPE_DS);
    // ZMP, CoM are at [0;0]


    // all subsequent steps have normal feet size
    d[0] = 0.09;
    d[1] = 0.025;
    d[2] = 0.03;
    d[3] = 0.025;
    // 2 reference ZMP positions in single support 
    // 1 in double support
    // 1 + 2 = 3
    wmg->AddFootstep(0.0   , -step_y/2, 0.0 , 5,  6, d);
    wmg->AddFootstep(step_x,  step_y, 0.0);
    wmg->AddFootstep(step_x, -step_y, 0.0);
    wmg->AddFootstep(step_x,  step_y, 0.0);
    wmg->AddFootstep(step_x, -step_y, 0.0);
    wmg->AddFootstep(step_x,  step_y, 0.0);

    // here we give many reference points, since otherwise we 
    // would not have enough steps in preview window to reach 
    // the last footsteps
    d[0] = 0.09;
    d[1] = 0.075;
    d[2] = 0.03;
    d[3] = 0.075;
    wmg->AddFootstep(0.0   , -step_y/2, 0.0, 30, 30, d, FS_TYPE_DS);
    d[0] = 0.09;
    d[1] = 0.025;
    d[2] = 0.03;
    d[3] = 0.025;
    wmg->AddFootstep(0.0   , -step_y/2, 0.0 , 0,  0, d, FS_TYPE_SS_R);
}



/**
 * @brief Walk straight
 */
void init_07 (WMG *wmg)
{
    double d[4];
    wmg->init(40);

    // each step is defined relatively to the previous step
    double step_x = 0.035;      // relative X position
    double step_y = 0.1;       // relative Y position


    d[0] = 0.09;
    d[1] = 0.025;
    d[2] = 0.03;
    d[3] = 0.025;
    wmg->AddFootstep(0.0, step_y/2, 0.0, 0, 0, d, FS_TYPE_SS_L);

    // Initial double support
    d[0] = 0.09;
    d[1] = 0.075;
    d[2] = 0.03;
    d[3] = 0.075;
    wmg->AddFootstep(0.0, -step_y/2, 0.0, 10, 10, d, FS_TYPE_DS);
    // ZMP, CoM are at [0;0]


    // all subsequent steps have normal feet size
    d[0] = 0.09;
    d[1] = 0.025;
    d[2] = 0.03;
    d[3] = 0.025;
    // 2 reference ZMP positions in single support 
    // 1 in double support
    // 1 + 2 = 3
    wmg->AddFootstep(0.0   , -step_y/2, 0.0 , 20,  25, d);
    wmg->AddFootstep(step_x,  step_y, 0.0);
    wmg->AddFootstep(step_x, -step_y, 0.0);
    wmg->AddFootstep(step_x,  step_y, 0.0);
    wmg->AddFootstep(step_x, -step_y, 0.0);
    wmg->AddFootstep(step_x,  step_y, 0.0);
    wmg->AddFootstep(step_x, -step_y, 0.0);
    wmg->AddFootstep(step_x,  step_y, 0.0);
    wmg->AddFootstep(step_x, -step_y, 0.0);
    wmg->AddFootstep(step_x,  step_y, 0.0);

    // here we give many reference points, since otherwise we 
    // would not have enough steps in preview window to reach 
    // the last footsteps
    d[0] = 0.09;
    d[1] = 0.075;
    d[2] = 0.03;
    d[3] = 0.075;
    wmg->AddFootstep(0.0   , -step_y/2, 0.0, 140, 140, d, FS_TYPE_DS);
    d[0] = 0.09;
    d[1] = 0.025;
    d[2] = 0.03;
    d[3] = 0.025;
    wmg->AddFootstep(0.0   , -step_y/2, 0.0 , 0,  0, d, FS_TYPE_SS_R);
}



/**
 * @brief Walk straight
 */
void init_08 (WMG *wmg)
{
    double d[4];
    wmg->init(40);

    // each step is defined relatively to the previous step
    double step_x = 0.035;      // relative X position
    double step_y = 0.1;       // relative Y position


    d[0] = 0.09;
    d[1] = 0.025;
    d[2] = 0.03;
    d[3] = 0.025;
    wmg->AddFootstep(0.0, step_y/2, 0.0, 0, 0, d, FS_TYPE_SS_L);

    // Initial double support
    d[0] = 0.09;
    d[1] = 0.075;
    d[2] = 0.03;
    d[3] = 0.075;
    wmg->AddFootstep(0.0, -step_y/2, 0.0, 10, 10, d, FS_TYPE_DS);
    // ZMP, CoM are at [0;0]


    // all subsequent steps have normal feet size
    d[0] = 0.09;
    d[1] = 0.025;
    d[2] = 0.03;
    d[3] = 0.025;
    // 2 reference ZMP positions in single support 
    // 1 in double support
    // 1 + 2 = 3
    wmg->AddFootstep(0.0   , -step_y/2, 0.0 , 20,  25, d);
    wmg->AddFootstep(step_x,  step_y, 0.0);
    wmg->AddFootstep(step_x, -step_y, 0.0);
    wmg->AddFootstep(step_x,  step_y, 0.0);
    wmg->AddFootstep(step_x, -step_y, 0.0);
    wmg->AddFootstep(step_x,  step_y, 0.0);

    // here we give many reference points, since otherwise we 
    // would not have enough steps in preview window to reach 
    // the last footsteps
    d[0] = 0.09;
    d[1] = 0.075;
    d[2] = 0.03;
    d[3] = 0.075;
    wmg->AddFootstep(0.0   , -step_y/2, 0.0, 60, 60, d, FS_TYPE_DS);
    d[0] = 0.09;
    d[1] = 0.025;
    d[2] = 0.03;
    d[3] = 0.025;
    wmg->AddFootstep(0.0   , -step_y/2, 0.0 , 0,  0, d, FS_TYPE_SS_R);
}



void initNaoModel (nao_igm* nao)
{
    // joint angles
    nao->state.q[L_HIP_YAW_PITCH]  =  0.0;
    nao->state.q[R_HIP_YAW_PITCH]  =  0.0;

    nao->state.q[L_HIP_ROLL]       = -0.000384;
    nao->state.q[L_HIP_PITCH]      = -0.598291;
    nao->state.q[L_KNEE_PITCH]     =  1.009413;
    nao->state.q[L_ANKLE_PITCH]    = -0.492352;
    nao->state.q[L_ANKLE_ROLL]     =  0.000469;

    nao->state.q[R_HIP_ROLL]       = -0.000384;
    nao->state.q[R_HIP_PITCH]      = -0.598219;
    nao->state.q[R_KNEE_PITCH]     =  1.009237;
    nao->state.q[R_ANKLE_PITCH]    = -0.492248;
    nao->state.q[R_ANKLE_ROLL]     =  0.000469;

    nao->state.q[L_SHOULDER_PITCH] =  1.418908;
    nao->state.q[L_SHOULDER_ROLL]  =  0.332836;
    nao->state.q[L_ELBOW_YAW]      = -1.379108;
    nao->state.q[L_ELBOW_ROLL]     = -1.021602;
    nao->state.q[L_WRIST_YAW]      = -0.013848;

    nao->state.q[R_SHOULDER_PITCH] =  1.425128;
    nao->state.q[R_SHOULDER_ROLL]  = -0.331386;
    nao->state.q[R_ELBOW_YAW]      =  1.383626;
    nao->state.q[R_ELBOW_ROLL]     =  1.029356;
    nao->state.q[R_WRIST_YAW]      = -0.01078;

    nao->state.q[HEAD_PITCH]       =  0.0;
    nao->state.q[HEAD_YAW]         =  0.0;



    // support foot position and orientation
    /// @attention Hardcoded parameters.
    nao->init (
            IGM_SUPPORT_RIGHT,
            0.0, -0.05, 0.0,
            0.0, 0.0, 0.0);
}
