#ifndef SENSOR_MODEL_HPP
#define SENSOR_MODEL_HPP

#include "map_reader.hpp"
#include <vector>

namespace pfilter
{
    class SensorModel
    {
       public:
           using Ray = std::vector<cv::Point2i>;
           using Rays = std::vector<Ray>;

        SensorModel(pfilter::MapReader& r_map_reader, size_t log_file_num);
        virtual ~SensorModel();

        double gaussian(double actual_range, double measured_range);
        double expon(double actual_range, double measured_range);
        double uniform(double measured_range);
        double max_range(double measured_range);

        double raycast(const cv::Vec3d& x_t, unsigned int theta_radian, Ray& ray);
        cv::Mat testRaycast(cv::Mat world_free_map, const Rays& rays);
        double beamRangeFinderModel(const std::vector<double>& z_t, const cv::Vec3d& x_t, bool test_raycast);

       private:
        const std::vector<double> ray_max_table = { 818.3, 819.1, 819.1, 819.1, 819.1 };
        pfilter::MapReader& map_reader_;
        double ray_max_;
        double occupancy_threshold_;
        double z_hit_;
        double z_short_;
        double z_max_;
        double z_rand_;

        double gaussian_variance_;
        double lambda_short_;
    };
} /* namespace pfilter */
#endif /* SENSOR_MODEL_HPP */
