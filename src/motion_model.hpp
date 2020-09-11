#ifndef MOTION_MODEL_HPP
#define MOTION_MODEL_HPP

#include <random>
#include <opencv2/opencv.hpp>

namespace pfilter
{
    class MotionModel
    {
    private:
        int map_resolution;
        double alpha_1;
        double alpha_2;
        double alpha_3;
        double alpha_4;
        double motion_threshold;

        double odo_rot1_delta;
        double odo_rot2_delta;
        double odo_trans_delta;

        std::default_random_engine generator;
    public:
        MotionModel(int resolution);
        virtual ~MotionModel();

        bool isMoving(const cv::Vec3d& u_t0, const cv::Vec3d& u_t1);
        inline double distanceMoved(void) { return odo_trans_delta; };
        cv::Vec3d update(cv::Vec3d u_t0, cv::Vec3d u_t1, cv::Vec3d x_t0);

    };
} /* namespace pfilter */
#endif /* MOTION_MODEL_HPP */
