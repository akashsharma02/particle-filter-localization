#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

#include <opencv2/opencv.hpp>

namespace pfilter
{
    class MotionModel
    {
    private:
        int map_resolution;
        float alpha_1;
        float alpha_2;
        float alpha_3;
        float alpha_4;
        double motion_threshold;

    public:
        MotionModel(int resolution);
        virtual ~MotionModel();

        bool isMoving(cv::Vec3d u_t0, cv::Vec3d u_t1);
        cv::Vec3d update(cv::Vec3d u_t0, cv::Vec3d u_t1, cv::Vec3d x_t0);
    };
} /* namespace pfilter */
#endif /* MOTION_MODEL_H */
