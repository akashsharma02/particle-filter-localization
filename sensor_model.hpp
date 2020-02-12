#ifndef SENSOR_MODEL_HPP
#define SENSOR_MODEL_HPP

#include "map_reader.hpp"

namespace pfilter
{
    class SensorModel
    {
    private:
        const std::vector<double> ray_max_table = {818.3, 819.1, 819.1, 819.1, 819.1};
        pfilter::MapReader& mr_map_reader;
        double ray_max;
        double occupancy_threshold;
        double z_hit;
        double z_short;
        double z_max;
        double z_rand;

        double gaussian_variance;
        double lambda_short;


    public:
        SensorModel(pfilter::MapReader& r_map_reader, int log_file_num);
        virtual ~SensorModel();

        double gaussian(double actual_range, double measured_range);
        double expon(double actual_range, double measured_range);
        double uniform(double actual_range, double measured_range);
        double max_range(double actual_range, double measured_range);

        double raycast(const cv::Vec3d& x_t, int ray_theta, std::vector<cv::Point2i>& r_ray_points);
        cv::Mat testRaycast(cv::Mat world_free_map, const std::vector<std::vector<cv::Point2i>>& r_lines);
        double beamRangeFinderModel(const std::vector<double>& z_t, const cv::Vec3d& x_t, bool test_raycast);
    };
} /* namespace pfilter */
#endif /* SENSOR_MODEL_HPP */
