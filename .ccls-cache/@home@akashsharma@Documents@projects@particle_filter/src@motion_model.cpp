#include "../include/motion_model.hpp"

#include <cmath>
#include <random>

namespace pfilter
{
    MotionModel::MotionModel(int resolution)
        : map_resolution(resolution)
        , alpha_1(0.0001)
        , alpha_2(0.0001)
        , alpha_3(0.0001)
        , alpha_4(0.0001)
        , motion_threshold(0.5) // 1cm
    {
    }

    MotionModel::~MotionModel()
    {
        //TODO: Delete allocated memory
    }

    cv::Vec3d MotionModel::update(cv::Vec3d u_t0, cv::Vec3d u_t1, cv::Vec3d x_t0)
    {
        double x0 = u_t0[0];
        double y0 = u_t0[1];
        double theta0 = u_t0[2];

        double x1 = u_t1[0];
        double y1 = u_t1[1];
        double theta1 = u_t1[2];

        double odo_rot1_delta = std::atan2(y1 - y0, x1 - x0) - theta0;
        double odo_trans_delta = std::sqrt(std::pow(x1 - x0, 2) + std::pow(y1 - y0, 2));
        double odo_rot2_delta = theta1 - theta0 - odo_rot1_delta;

        std::normal_distribution<double> rot1_dist(0, std::sqrt(alpha_1*(std::pow(odo_rot1_delta, 2)) + alpha_2*(std::pow(odo_trans_delta, 2))));
        std::normal_distribution<double> trans_dist(0, std::sqrt(alpha_3*(std::pow(odo_trans_delta, 2)) + alpha_4*(std::pow(odo_rot1_delta, 2) + std::pow(odo_rot2_delta, 2))));
        std::normal_distribution<double> rot2_dist(0, std::sqrt(alpha_1*(std::pow(odo_rot2_delta, 2)) + alpha_2*(std::pow(odo_trans_delta, 2))));

        double rot1_delta = odo_rot1_delta - rot1_dist(generator);
        double trans_delta = odo_trans_delta - trans_dist(generator);
        double rot2_delta = odo_rot2_delta - rot2_dist(generator);

        cv::Vec3d x_t1;
        x_t1[0] = x_t0[0] + trans_delta * cos(x_t0[2] + rot1_delta);
        x_t1[1] = x_t0[1] + trans_delta * sin(x_t0[2] + rot1_delta);
        x_t1[2] = x_t0[2] + rot1_delta + rot2_delta;

        return x_t1;
    }

    bool MotionModel::isMoving(cv::Vec3d u_t0, cv::Vec3d u_t1)
    {
        double x0 = u_t0[0] / map_resolution;
        double y0 = u_t0[1] / map_resolution;
        double theta0 = u_t0[2];

        double x1 = u_t1[0] / map_resolution;
        double y1 = u_t1[1] / map_resolution;
        double theta1 = u_t1[2];

        double translation_delta = std::sqrt(std::pow(x1 - x0, 2) + std::pow(y1 - y0, 2));
        /* std::cout << u_t0 << " " << u_t1 << "distance: " << translation_delta << std::endl; */
        if(translation_delta < motion_threshold)
            return false;
        return true;
    }

} /* namespace pfilter */
