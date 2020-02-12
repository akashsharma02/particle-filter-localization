#include "sensor_model.hpp"

#include <cmath>

namespace pfilter
{
    SensorModel::SensorModel(pfilter::MapReader& r_map_reader, int log_file_num)
        : mr_map_reader(r_map_reader)
        , ray_max(ray_max_table.at(log_file_num))
        , occupancy_threshold(0.5)
        , z_hit(1600.0)
        , z_short(1800.0)
        , z_max(600.0)
        , z_rand(250.0)
        , gaussian_variance(90.0) //30 pixels of stddev
        , lambda_short(100.0)
    {


    }

    SensorModel::~SensorModel()
    {
    }

    double SensorModel::gaussian(double actual_range, double measured_range)
    {
        if(measured_range >= 0 && measured_range < ray_max)
        {
            double normalizer = std::sqrt(2*CV_PI*gaussian_variance);
            double exponential = std::exp(-std::pow(measured_range - actual_range, 2)/(2 * gaussian_variance));
            return (exponential / normalizer);
        }
        return 0;
    }

    double SensorModel::expon(double actual_range, double measured_range)
    {
        if(measured_range >= 0 && measured_range < actual_range)
        {
            double exponential = lambda_short * std::exp(-lambda_short * measured_range);
            return exponential;
        }
        return 0;
    }

    double SensorModel::uniform(double actual_range, double measured_range)
    {
        if(measured_range >= 0 && measured_range < ray_max)
        {
            return 1/ray_max;
        }
        return 0;
    }

    double SensorModel::max_range(double actual_range, double measured_range)
    {
        if(measured_range == ray_max)
            return 1;
        return 0;
    }

    /*
        Implement bresenham's line algorithm to raycast
    */
    double SensorModel::raycast(const cv::Vec3d& x_t, int ray_theta, std::vector<cv::Point2i>& r_ray_points)
    {
        double theta = x_t[2] - CV_PI/2 + ray_theta*CV_PI/180;

        //Add 25 to the robot coordinates to account for laser position
        int x1 = static_cast<int>(x_t[0]/mr_map_reader.resolution + 2.5*cos(theta));
        int y1 = static_cast<int>(x_t[1]/mr_map_reader.resolution + 2.5*sin(theta));

        int x2 = static_cast<int>(x1 + ray_max * cos(theta));
        int y2 = static_cast<int>(y1 + ray_max * sin(theta));

        double dx = abs(x2 - x1);
        double dy = abs(y2 - y1);
        int sign_x = x1 > x2? -1: 1;
        int sign_y = y1 > y2? -1: 1;
        int x = x1;
        int y = y1;

        cv::Mat occupancy_map = mr_map_reader.getOccupancyMap();
        if(dx > dy)
        {
            double error = dx/2.0;
            while(x != x2)
            {
                if(x > mr_map_reader.size_x/mr_map_reader.resolution - 1 || x < 0)
                    break;
                if(y > mr_map_reader.size_y/mr_map_reader.resolution - 1 || y < 0)
                    break;
                cv::Point2i point = cv::Point2i(x, y);
                if(occupancy_map.at<double>(point) > occupancy_threshold)
                    break;
                r_ray_points.push_back(point);
                error -= dy;
                if(error < 0)
                {
                    y += sign_y;
                    error += dx;
                }
                x += sign_x;
            }
            double distance = std::sqrt(std::pow(x - x1, 2) + std::pow(y - y1, 2));
            return distance;
        }
        double error = dy/2.0;
        while(y != y2)
        {
            if(x > mr_map_reader.size_x/mr_map_reader.resolution - 1 || x < 0)
                break;
            if(y > mr_map_reader.size_y/mr_map_reader.resolution - 1 || y < 0)
                break;
            cv::Point2i point = cv::Point2i(x, y);
            if(occupancy_map.at<double>(point) > occupancy_threshold)
                break;
            r_ray_points.push_back(point);
            error -= dx;
            if(error < 0)
            {
                x += sign_x;
                error += dy;
            }
            y += sign_y;
        }
        double distance = std::sqrt(std::pow(x - x1, 2) + std::pow(y - y1, 2));
        return distance;
    }

    cv::Mat SensorModel::testRaycast(cv::Mat world_free_map, const std::vector<std::vector<cv::Point2i>>& r_lines)
    {
        cv::Mat image, image_color, image_color_flipped;
        world_free_map.convertTo(image, CV_8UC3, 255);
        cv::cvtColor(image, image_color, cv::COLOR_GRAY2BGR);
        for (auto line : r_lines)
        {
            if(line.size() > 0)
                cv::line(image_color, line[0], line[line.size()-1], cv::Scalar(128, 128, 0));
        }
        cv::flip(image_color, image_color_flipped, 0);
        return image_color_flipped;
    }

    double SensorModel::beamRangeFinderModel(const std::vector<double> &z_t, const cv::Vec3d &x_t, bool test_raycast)
    {
        double q = 0;
        double max_p = 0;
        //Sample from the range sensor every 20 degrees only
        std::vector<std::vector<cv::Point2i>> lines;
        for(unsigned int i = 0; i < z_t.size(); i += 20)
        {
            std::vector<cv::Point2i> line_points;
            double actual_range = raycast(x_t, i, line_points);
            lines.push_back(line_points);
            double measured_range = z_t[i]/mr_map_reader.resolution;
            double gauss = z_hit * gaussian(actual_range, measured_range);
            double exp   = z_short * expon(actual_range, measured_range);
            double uni   = z_rand * uniform(actual_range, measured_range);
            double max   = z_max * max_range(actual_range, measured_range);
            //NOTE: Use log likelihood for numerical stability
            double p = log(gauss + exp + uni + max);
            if(p > max_p)
                max_p = p;
            q = q + p;
        }
        if(lines.size() > 0 && test_raycast)
        {
            cv::Mat image = testRaycast(mr_map_reader.getFreeMap(), lines);
            cv::imshow("Map", image);
            while(cv::waitKey(0) != 27);
        }
        //NOTE: subtract max log likelihood for numerical stability
        q = q - max_p;
        return q;
    }
} /* namespace pfilter */
