#include <cmath>


void initNaoModel (nao_igm& nao, double* ref_angles)
{
    // joint angles
    nao.state_sensor.q[L_HIP_YAW_PITCH] = ref_angles[L_HIP_YAW_PITCH] =  0.0;
    nao.state_sensor.q[R_HIP_YAW_PITCH] = ref_angles[R_HIP_YAW_PITCH] =  0.0;
                                                             
    nao.state_sensor.q[L_HIP_ROLL]      = ref_angles[L_HIP_ROLL]      = -0.000384;
    nao.state_sensor.q[L_HIP_PITCH]     = ref_angles[L_HIP_PITCH]     = -0.598291;
    nao.state_sensor.q[L_KNEE_PITCH]    = ref_angles[L_KNEE_PITCH]    =  1.009413;
    nao.state_sensor.q[L_ANKLE_PITCH]   = ref_angles[L_ANKLE_PITCH]   = -0.492352;
    nao.state_sensor.q[L_ANKLE_ROLL]    = ref_angles[L_ANKLE_ROLL]    =  0.000469;
                                                             
    nao.state_sensor.q[R_HIP_ROLL]      = ref_angles[R_HIP_ROLL]      = -0.000384;
    nao.state_sensor.q[R_HIP_PITCH]     = ref_angles[R_HIP_PITCH]     = -0.598219;
    nao.state_sensor.q[R_KNEE_PITCH]    = ref_angles[R_KNEE_PITCH]    =  1.009237;
    nao.state_sensor.q[R_ANKLE_PITCH]   = ref_angles[R_ANKLE_PITCH]   = -0.492248;
    nao.state_sensor.q[R_ANKLE_ROLL]    = ref_angles[R_ANKLE_ROLL]    =  0.000469;

    nao.state_sensor.q[L_SHOULDER_PITCH] =  1.418908;
    nao.state_sensor.q[L_SHOULDER_ROLL]  =  0.332836;
    nao.state_sensor.q[L_ELBOW_YAW]      = -1.379108;
    nao.state_sensor.q[L_ELBOW_ROLL]     = -1.021602;
    nao.state_sensor.q[L_WRIST_YAW]      = -0.013848;

    nao.state_sensor.q[R_SHOULDER_PITCH] =  1.425128;
    nao.state_sensor.q[R_SHOULDER_ROLL]  = -0.331386;
    nao.state_sensor.q[R_ELBOW_YAW]      =  1.383626;
    nao.state_sensor.q[R_ELBOW_ROLL]     =  1.029356;
    nao.state_sensor.q[R_WRIST_YAW]      = -0.01078;

    nao.state_sensor.q[HEAD_PITCH]       =  0.0;
    nao.state_sensor.q[HEAD_YAW]         =  0.0;
}



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



        smpc::state_zmp X_tilde;
        WMG* wmg;
        smpc_parameters* par;

        nao_igm nao;
        double ref_angles[LOWER_JOINTS_NUM];

        string name;
        string fs_out_filename;
        bool plot_ds;
};



/**
 * @brief Walk straight
 */
class init_08 : public test_init_base
{
    public:
        init_08 (
                const string & test_name, 
                const int preview_sampling_time_ms,
                const bool plot_ds_ = true) :
            test_init_base (test_name, plot_ds_)
        {
            initNaoModel (nao, ref_angles);
            // support foot position and orientation
            nao.init (
                    IGM_SUPPORT_RIGHT,
                    0.0, -0.05, 0.0,
                    0.0, 0.0, 0.0);
            nao.getCoM(nao.state_sensor, nao.CoM_position);

            wmg = new WMG (40, preview_sampling_time_ms, 0.02);
            par = new smpc_parameters (wmg->N, nao.CoM_position[2]);
            int ss_time_ms = 400;
            int ds_time_ms = 40;
            int ds_number = 3;

            // each step is defined relatively to the previous step
            double step_x = 0.04;      // relative X position
            double step_y = wmg->def_constraints.support_distance_y;       // relative Y position

/*

            wmg->setFootstepParametersMS (0, 0, 0);
            wmg->addFootstep(0.0, -step_y/2, 0.0, FS_TYPE_SS_R);

            // Initial double support
            wmg->setFootstepParametersMS (3*ss_time_ms, 0, 0);
            wmg->addFootstep(0.0, step_y/2, 0.0, FS_TYPE_DS);


            // all subsequent steps have normal feet size
            wmg->setFootstepParametersMS (ss_time_ms, 0, 0);
            wmg->addFootstep(0.0   , step_y/2, 0.0);
            wmg->setFootstepParametersMS (ss_time_ms, ds_time_ms, ds_number);
            wmg->addFootstep(step_x,  -step_y, 0.0);


            for (int i = 0; i < 3; i++)
            {
                wmg->addFootstep(step_x,  step_y, 0.0);
                wmg->addFootstep(step_x, -step_y, 0.0);
            }

            // here we give many reference points, since otherwise we 
            // would not have enough steps in preview window to reach 
            // the last footsteps
            wmg->setFootstepParametersMS (5*ss_time_ms, 0, 0);
            wmg->addFootstep(0.0   , step_y/2, 0.0, FS_TYPE_DS);
            wmg->setFootstepParametersMS (0, 0, 0);
            wmg->addFootstep(0.0   , step_y/2, 0.0, FS_TYPE_SS_L);
*/
    wmg->setFootstepParametersMS(0, 0, 0);
    wmg->addFootstep(0.0, step_y/2, 0.0, FS_TYPE_SS_L);

    // Initial double support
    wmg->setFootstepParametersMS(3*ss_time_ms, 0, 0);
    wmg->addFootstep(0.0, -step_y/2, 0.0, FS_TYPE_DS);


    // all subsequent steps have normal feet size
    wmg->setFootstepParametersMS(ss_time_ms, 0, 0);
    wmg->addFootstep(0.0   , -step_y/2, 0.0);
    wmg->setFootstepParametersMS(ss_time_ms, ds_time_ms, ds_number);
    wmg->addFootstep(step_x,  step_y,   0.0);

    for (int i = 0; i < 4; i++)
    {
        wmg->addFootstep(step_x, -step_y, 0.0);
        wmg->addFootstep(step_x,  step_y, 0.0);
    }

    // here we give many reference points, since otherwise we 
    // would not have enough steps in preview window to reach 
    // the last footsteps
    wmg->setFootstepParametersMS(5*ss_time_ms, 0, 0);
    wmg->addFootstep(0.0   , -step_y/2, 0.0, FS_TYPE_DS);
    wmg->setFootstepParametersMS(0, 0, 0);
    wmg->addFootstep(0.0   , -step_y/2, 0.0, FS_TYPE_SS_R);



            if (!name.empty())
            {
                wmg->FS2file(fs_out_filename, plot_ds);
            }
        }
};



/**
 * @brief Diagonal walk
 */
class init_09 : public test_init_base
{
    public:
        init_09 (
                const string & test_name, 
                const int preview_sampling_time_ms,
                const bool plot_ds_ = true) :
            test_init_base (test_name, plot_ds_)
        {
            initNaoModel (nao, ref_angles);
            // support foot position and orientation
            nao.init (
                    IGM_SUPPORT_LEFT,
                    0.0, 0.05, 0.0,
                    0.0, 0.0, 0.0);
            nao.getCoM(nao.state_sensor, nao.CoM_position);

            wmg = new WMG (40, preview_sampling_time_ms, 0.02);
            par = new smpc_parameters (wmg->N, nao.CoM_position[2]);
            int ss_time_ms = 400;
            int ds_time_ms = 40;
            int ds_number = 3;


            // each step is defined relatively to the previous step
            double step_x = 0.04;      // relative X position
            double step_y = wmg->def_constraints.support_distance_y;       // relative Y position

            wmg->setFootstepParametersMS (0, 0, 0);
            wmg->addFootstep(0.0, step_y/2, 0.0, FS_TYPE_SS_L);

            // Initial double support
            wmg->setFootstepParametersMS (3*ss_time_ms, 0, 0);
            wmg->addFootstep(0.0, -step_y/2, 0.0, FS_TYPE_DS);

            // each step is defined relatively to the previous step
            double shift = -0.02;

            // all subsequent steps have normal feet size
            wmg->setFootstepParametersMS (ss_time_ms, ds_time_ms, ds_number);
            wmg->addFootstep(0.0   , -step_y/2, 0.0);
            wmg->addFootstep(step_x,  step_y + shift, 0.0);
            wmg->addFootstep(step_x, -step_y + shift, 0.0);
            wmg->addFootstep(step_x,  step_y + shift, 0.0);
            wmg->addFootstep(step_x, -step_y + shift, 0.0);
            wmg->addFootstep(step_x,  step_y + shift, 0.0);
            wmg->addFootstep(step_x, -step_y + shift, 0.0);
            wmg->addFootstep(step_x,  step_y + shift, 0.0);
            wmg->addFootstep(step_x, -step_y + shift, 0.0);
            wmg->addFootstep(step_x,  step_y + shift, 0.0);


            // here we give many reference points, since otherwise we 
            // would not have enough steps in preview window to reach 
            // the last footsteps
            wmg->setFootstepParametersMS (5*ss_time_ms, 0, 0);
            wmg->addFootstep(0.0   , -step_y/2, 0.0, FS_TYPE_DS);
            wmg->setFootstepParametersMS (0, 0, 0);
            wmg->addFootstep(0.0   , -step_y/2, 0.0, FS_TYPE_SS_R);

            if (!name.empty())
            {
                wmg->FS2file(fs_out_filename, plot_ds);
            }
        }
};



/**
 * @brief Circular walk. // Doesn't work on the robot.
 */
class init_10 : public test_init_base
{
    public:
        init_10 (
                const string & test_name, 
                const int preview_sampling_time_ms,
                const bool plot_ds_ = true) :
            test_init_base (test_name, plot_ds_)
        {
            initNaoModel (nao, ref_angles);
            // support foot position and orientation
            nao.init (
                    IGM_SUPPORT_LEFT,
                    0.0, 0.05, 0.0,
                    0.0, 0.0, 0.0);
            nao.getCoM(nao.state_sensor, nao.CoM_position);

            wmg = new WMG (40, preview_sampling_time_ms, 0.02);
            par = new smpc_parameters (wmg->N, nao.CoM_position[2]);
            int ss_time_ms = 400;
            int ds_time_ms = 40;
            int ds_number = 3;



            // each step is defined relatively to the previous step
            double step_x_ext = 0.04;      // relative X position
            double step_y = wmg->def_constraints.support_distance_y;       // relative Y position


            double R_ext = 0.55;
            double R_int = R_ext - step_y;

            // relative angle
            double a = asin (step_x_ext / R_ext);
            double step_x_int = step_x_ext * R_int / R_ext;
//            double step_x_int = step_x_ext;


            wmg->setFootstepParametersMS (0, 0, 0);
            wmg->addFootstep(0.0, step_y/2, 0.0, FS_TYPE_SS_L);

            // Initial double support
            wmg->setFootstepParametersMS (3*ss_time_ms, 0, 0);
            wmg->addFootstep(0.0, -step_y/2, 0.0, FS_TYPE_DS);



            wmg->setFootstepParametersMS (ss_time_ms, ds_time_ms, ds_number);
            wmg->addFootstep(0.0   ,     -step_y/2, 0.0);
            wmg->addFootstep(step_x_int,  step_y, a);

            for (int i = 0; i < 10; i++)
            {
                wmg->addFootstep(step_x_ext, -step_y, a);
                wmg->addFootstep(step_x_int,  step_y, a);
            }

            // here we give many reference points, since otherwise we 
            // would not have enough steps in preview window to reach 
            // the last footsteps
            wmg->setFootstepParametersMS (6*ss_time_ms, 0, 0);
            wmg->addFootstep(0.0   , -step_y/2, 0.0, FS_TYPE_DS);
            wmg->setFootstepParametersMS (0, 0, 0);
            wmg->addFootstep(0.0   , -step_y/2, 0.0, FS_TYPE_SS_R);


            if (!name.empty())
            {
                wmg->FS2file(fs_out_filename, plot_ds);
            }
        }
};



class init_11 : public test_init_base
{
    public:
        init_11 (
                const string & test_name, 
                const int preview_sampling_time_ms,
                const bool plot_ds_ = true) :
            test_init_base (test_name, plot_ds_)
        {
            initNaoModel (nao, ref_angles);
            // support foot position and orientation
            nao.init (
                    IGM_SUPPORT_LEFT,
                    0.0, 0.05, 0.0,
                    0.0, 0.0, 0.0);
            nao.getCoM(nao.state_sensor, nao.CoM_position);

            wmg = new WMG (40, preview_sampling_time_ms, 0.02);
            par = new smpc_parameters (wmg->N, nao.CoM_position[2]);
            int ss_time_ms = 400;
            int ds_time_ms = 40;
            int ds_number = 3;

            // each step is defined relatively to the previous step
            double step_x = 0.04;      // relative X position
            double step_y = wmg->def_constraints.support_distance_y;       // relative Y position


            wmg->setFootstepParametersMS(0, 0, 0);
            wmg->addFootstep(0.0, step_y/2, 0.0, FS_TYPE_SS_L);

            // Initial double support
            wmg->setFootstepParametersMS(3*ss_time_ms, 0, 0);
            wmg->addFootstep(0.0, -step_y/2, 0.0, FS_TYPE_DS);


            // all subsequent steps have normal feet size
            wmg->setFootstepParametersMS(ss_time_ms, 0, 0);
            wmg->addFootstep(0.0   , -step_y/2, 0.0);
            wmg->setFootstepParametersMS(ss_time_ms, ds_time_ms, ds_number);
            wmg->addFootstep(step_x,  step_y,   0.0);

            for (int i = 0; i < 2; i++)
            {
                wmg->addFootstep(step_x, -step_y, 0.0);
                wmg->addFootstep(step_x,  step_y, 0.0);
            }

            // here we give many reference points, since otherwise we 
            // would not have enough steps in preview window to reach 
            // the last footsteps
            wmg->setFootstepParametersMS(5*ss_time_ms, 0, 0);
            wmg->addFootstep(0.0   , -step_y/2, 0.0, FS_TYPE_DS);
            wmg->setFootstepParametersMS(0, 0, 0);
            wmg->addFootstep(0.0   , -step_y/2, 0.0, FS_TYPE_SS_R);

            if (!name.empty())
            {
                wmg->FS2file(fs_out_filename, plot_ds);
            }
        }
};
