#ifndef PFILTER_MOTION_MODEL_HPP
#define PFILTER_MOTION_MODEL_HPP

#include <opencv2/opencv.hpp>
#include <random>

namespace pfilter
{
    class MotionModel
    {
       public:
        explicit MotionModel(int resolution);
        virtual ~MotionModel();

        bool isMoving(const cv::Vec3d& u_t0, const cv::Vec3d& u_t1) const;
        inline double distanceMoved() const { return odo_trans_delta_; };
        cv::Vec3d update(const cv::Vec3d& u_t0, const cv::Vec3d& u_t1, const cv::Vec3d& x_t0);

       private:
        int map_resolution_;
        double alpha_1_;
        double alpha_2_;
        double alpha_3_;
        double alpha_4_;
        double motion_threshold_;

        double odo_rot1_delta_;
        double odo_rot2_delta_;
        double odo_trans_delta_;

        std::default_random_engine generator_;
    };
} /* namespace pfilter */
#endif /* PFILTER_MOTION_MODEL_HPP */
