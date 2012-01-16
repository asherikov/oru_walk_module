/**
 * @file
 * @author Alexander Sherikov
 */

#include "oru_walk.h"



walk_parameters::walk_parameters()
{
    feedback_gain = 0.3;
    feedback_threshold = 0.006;


    mpc_alpha = 1000.0;
    mpc_beta = 11000.0;
    mpc_gamma = 1.0;
    mpc_regularization = 0.01;
    mpc_tolerance = 1e-7;


    step_height = 0.013;
    /// 0.0135 in the old version of our module
    /// @ref AldNaoPaper "0.015 in the paper"
    /// 0.02 is used in the built-in module


    control_sampling_time_ms = 10;
    preview_sampling_time_ms = 20;
    preview_window_size = 40;


    filter_window_length = 1;
}



/**
 * @brief Changes specified parameters.
 *
 * @param[in] feedback_gain_ feedback gain
 * @param[in] feedback_threshold_ feedback threshold
 * @param[in] mpc_alpha_ alpha gain for the QP objective
 * @param[in] mpc_beta_ beta gain for the QP objective
 * @param[in] mpc_gamma_ gamma gain for the QP objective
 * @param[in] step_height_ height of a step
 */
void walk_parameters::set (
        const double feedback_gain_,
        const double feedback_threshold_,
        const double mpc_alpha_,
        const double mpc_beta_,
        const double mpc_gamma_,
        const double step_height_)
{
    feedback_gain = feedback_gain_;
    feedback_threshold = feedback_threshold_;

    mpc_alpha = mpc_alpha_;
    mpc_beta = mpc_beta_;
    mpc_gamma = mpc_gamma_;

    step_height = step_height_;
}
