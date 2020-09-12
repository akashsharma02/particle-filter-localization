#include "motion_model.hpp"

#include <cmath>
#include <random>

namespace pfilter
{
    MotionModel::MotionModel(int resolution)
        : map_resolution_(resolution),
          alpha_1_(0.0001),
          alpha_2_(0.0001),
          alpha_3_(0.0005),
          alpha_4_(0.0005),
          motion_threshold_(0.5), // 1cm
          odo_rot1_delta_(0.0),
          odo_rot2_delta_(0.0),
          odo_trans_delta_(0.0)
    {
    }

    MotionModel::~MotionModel()
    {
        // TODO: Delete allocated memory
    }

    cv::Vec3d MotionModel::update(const cv::Vec3d& u_t0, const cv::Vec3d& u_t1, const cv::Vec3d& x_t0)
    {
        double x0     = u_t0[0];
        double y0     = u_t0[1];
        double theta0 = u_t0[2];

        double x1     = u_t1[0];
        double y1     = u_t1[1];
        double theta1 = u_t1[2];

        odo_rot1_delta_  = std::atan2(y1 - y0, x1 - x0) - theta0;
        odo_trans_delta_ = std::sqrt(std::pow(x1 - x0, 2) + std::pow(y1 - y0, 2));
        odo_rot2_delta_  = theta1 - theta0 - odo_rot1_delta_;

        std::normal_distribution<double> rot1_dist(
            0, std::sqrt(alpha_1_ * (std::pow(odo_rot1_delta_, 2)) + alpha_2_ * (std::pow(odo_trans_delta_, 2))));
        std::normal_distribution<double> trans_dist(
            0,
            std::sqrt(alpha_3_ * (std::pow(odo_trans_delta_, 2)) +
                      alpha_4_ * (std::pow(odo_rot1_delta_, 2) + std::pow(odo_rot2_delta_, 2))));
        std::normal_distribution<double> rot2_dist(
            0, std::sqrt(alpha_1_ * (std::pow(odo_rot2_delta_, 2)) + alpha_2_ * (std::pow(odo_trans_delta_, 2))));

        double rot1_delta  = odo_rot1_delta_ - rot1_dist(generator_);
        double trans_delta = odo_trans_delta_ - trans_dist(generator_);
        double rot2_delta  = odo_rot2_delta_ - rot2_dist(generator_);

        cv::Vec3d x_t1;
        x_t1[0] = x_t0[0] + trans_delta * cos(x_t0[2] + rot1_delta);
        x_t1[1] = x_t0[1] + trans_delta * sin(x_t0[2] + rot1_delta);
        x_t1[2] = x_t0[2] + rot1_delta + rot2_delta;

        return x_t1;
    }

    bool MotionModel::isMoving(const cv::Vec3d& u_t0, const cv::Vec3d& u_t1) const
    {
        double x0     = u_t0[0] / map_resolution_;
        double y0     = u_t0[1] / map_resolution_;

        double x1     = u_t1[0] / map_resolution_;
        double y1     = u_t1[1] / map_resolution_;

        double translation_delta = std::sqrt(std::pow(x1 - x0, 2) + std::pow(y1 - y0, 2));
        if (translation_delta < motion_threshold_)
            return false;
        return true;
    }

} /* namespace pfilter */
