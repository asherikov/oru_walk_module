class test_init_base
{
    public:
        test_init_base(const string& test_name, const bool plot_ds_)
        {
            name = test_name;
            plot_ds = plot_ds_;

            if (!name.empty())
            {
                cout << "################################" << endl;
                cout << name << endl;
                cout << "################################" << endl;
                fs_out_filename = name + "_fs.m";
            }
        }
        ~test_init_base()
        {
            if (!name.empty())
            {
                cout << "################################" << endl;
            }
        }



        smpc::state_tilde X_tilde;
        WMG* wmg;
        smpc_parameters* par;

        string name;
        string fs_out_filename;
        bool plot_ds;
};



/**
 * @brief Walk straight
 */
class init_04 : public test_init_base
{
    public:
        init_04 (
                const string & test_name, 
                const int preview_sampling_time_ms,
                const double hCoM,
                const bool plot_ds_ = true) :
            test_init_base (test_name, plot_ds_)
        {
            wmg = new WMG (40, preview_sampling_time_ms, 0.0135);
            par = new smpc_parameters (wmg->N, hCoM);


            // each step is defined relatively to the previous step
            double step_x = 0.035;      // relative X position
            double step_y = 0.1;       // relative Y position


            double ds_constraint[4] = {
                wmg->def_ss_constraint[0],
                wmg->def_ss_constraint[1] + 0.5*step_y,
                wmg->def_ss_constraint[2],
                wmg->def_ss_constraint[3] + 0.5*step_y};


            wmg->AddFootstep(0.0, step_y/2, 0.0, 0, 0, wmg->def_ss_constraint, FS_TYPE_SS_L);

            // Initial double support
            wmg->AddFootstep(0.0, -step_y/2, 0.0, 10, 10, ds_constraint, FS_TYPE_DS);


            // 2 reference ZMP positions in single support 
            // 1 in double support
            // 1 + 2 = 3
            wmg->AddFootstep(0.0   , -step_y/2, 0.0 , 5,  6, wmg->def_ss_constraint);
            wmg->AddFootstep(step_x,  step_y, 0.0);
            wmg->AddFootstep(step_x, -step_y, 0.0);
            wmg->AddFootstep(step_x,  step_y, 0.0);
            wmg->AddFootstep(step_x, -step_y, 0.0);
            wmg->AddFootstep(step_x,  step_y, 0.0);

            // here we give many reference points, since otherwise we 
            // would not have enough steps in preview window to reach 
            // the last footsteps
            wmg->AddFootstep(0.0   , -step_y/2, 0.0, 120, 120, ds_constraint, FS_TYPE_DS);
            wmg->AddFootstep(0.0   , -step_y/2, 0.0 , 0,  0, wmg->def_ss_constraint, FS_TYPE_SS_R);

            if (!name.empty())
            {
                wmg->FS2file(fs_out_filename, plot_ds);
            }
        }
};



/**
 * @brief Walk straight
 */
class init_07 : public test_init_base
{
    public:
        init_07 (
                const string & test_name, 
                const int preview_sampling_time_ms,
                const double hCoM,
                const bool plot_ds_ = true) :
            test_init_base (test_name, plot_ds_)
        {
            wmg = new WMG (40, preview_sampling_time_ms, 0.02);
            par = new smpc_parameters (wmg->N, hCoM);
            int ss_number = 11;
            int ds_number = 3;


            // each step is defined relatively to the previous step
            double step_x = 0.04;      // relative X position
            double step_y = 0.1;       // relative Y position

            double ds_constraint[4] = {
                wmg->def_ss_constraint[0],
                wmg->def_ss_constraint[1] + 0.5*step_y,
                wmg->def_ss_constraint[2],
                wmg->def_ss_constraint[3] + 0.5*step_y};


            wmg->AddFootstep(0.0, step_y/2, 0.0, 0, 0, wmg->def_ss_constraint, FS_TYPE_SS_L);

            // Initial double support
            wmg->AddFootstep(0.0, -step_y/2, 0.0, 3*ss_number, 3*ss_number, ds_constraint, FS_TYPE_DS);
            // ZMP, CoM are at [0;0]


            // all subsequent steps have normal feet size
            // 2 reference ZMP positions in single support 
            // 1 in double support
            // 1 + 2 = 3
            wmg->AddFootstep(0.0   , -step_y/2, 0.0, ss_number, ss_number, wmg->def_ss_constraint);
            wmg->AddFootstep(step_x,  step_y, 0.0, ss_number, ss_number+ds_number, wmg->def_ss_constraint);
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
            wmg->AddFootstep(0.0   , -step_y/2, 0.0, 50, 50, ds_constraint, FS_TYPE_DS);
            wmg->AddFootstep(0.0   , -step_y/2, 0.0 , 0,  0, wmg->def_ss_constraint, FS_TYPE_SS_R);

            if (!name.empty())
            {
                wmg->FS2file(fs_out_filename, plot_ds);
            }
        }
};



void initNaoModel (nao_igm* nao)
{
    // joint angles
    nao->state_sensor.q[L_HIP_YAW_PITCH]  =  0.0;
    nao->state_sensor.q[R_HIP_YAW_PITCH]  =  0.0;

    nao->state_sensor.q[L_HIP_ROLL]       = -0.000384;
    nao->state_sensor.q[L_HIP_PITCH]      = -0.598291;
    nao->state_sensor.q[L_KNEE_PITCH]     =  1.009413;
    nao->state_sensor.q[L_ANKLE_PITCH]    = -0.492352;
    nao->state_sensor.q[L_ANKLE_ROLL]     =  0.000469;

    nao->state_sensor.q[R_HIP_ROLL]       = -0.000384;
    nao->state_sensor.q[R_HIP_PITCH]      = -0.598219;
    nao->state_sensor.q[R_KNEE_PITCH]     =  1.009237;
    nao->state_sensor.q[R_ANKLE_PITCH]    = -0.492248;
    nao->state_sensor.q[R_ANKLE_ROLL]     =  0.000469;

    nao->state_sensor.q[L_SHOULDER_PITCH] =  1.418908;
    nao->state_sensor.q[L_SHOULDER_ROLL]  =  0.332836;
    nao->state_sensor.q[L_ELBOW_YAW]      = -1.379108;
    nao->state_sensor.q[L_ELBOW_ROLL]     = -1.021602;
    nao->state_sensor.q[L_WRIST_YAW]      = -0.013848;

    nao->state_sensor.q[R_SHOULDER_PITCH] =  1.425128;
    nao->state_sensor.q[R_SHOULDER_ROLL]  = -0.331386;
    nao->state_sensor.q[R_ELBOW_YAW]      =  1.383626;
    nao->state_sensor.q[R_ELBOW_ROLL]     =  1.029356;
    nao->state_sensor.q[R_WRIST_YAW]      = -0.01078;

    nao->state_sensor.q[HEAD_PITCH]       =  0.0;
    nao->state_sensor.q[HEAD_YAW]         =  0.0;



    // support foot position and orientation
    /// @attention Hardcoded parameters.
    nao->init (
            IGM_SUPPORT_LEFT,
            0.0, 0.05, 0.0,
            0.0, 0.0, 0.0);
}
