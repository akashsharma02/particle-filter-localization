#include <iostream>
#include <fstream>
#include <random>
#include <opencv2/opencv.hpp>

#include "map_reader.h"
#include "motion_model.h"

void initParticles(int num_particles, pfilter::MapReader& r_map_reader, std::vector<cv::Vec3d>& r_particles, std::vector<double>& r_weights)
{
    cv::Mat occupancy_map = r_map_reader.getOccupancyMap();
    std::default_random_engine generator;
    std::uniform_int_distribution<> x_val_distribution(3000, 7000);
    std::uniform_int_distribution<> y_val_distribution(0, 8000);
    std::uniform_real_distribution<> theta_distribution(-3.14, 3.14);
    std::uniform_real_distribution<> u_distribution(0, 1);

    /* std::vector<cv::Vec3d> particles(num_particles); */
    std::vector<double> u_vals(num_particles*10);
    r_weights.resize(num_particles, 1/num_particles);

    // Rejection sampling until num_particles reached
    for(unsigned int i = 0; r_particles.size() < num_particles; i++)
    {
        cv::Vec3d particle;
        particle[0] = x_val_distribution(generator);
        particle[1] = y_val_distribution(generator);
        particle[2] = theta_distribution(generator);
        u_vals.at(i) = u_distribution(generator);

        cv::Point2i particle_point(particle[0]/r_map_reader.resolution, particle[1]/r_map_reader.resolution);
        if(u_vals.at(i) > occupancy_map.at<double>(particle_point) && occupancy_map.at<double>(particle_point) >= 0.0)
            r_particles.push_back(particle);
    }
}

cv::Mat plotParticles(int num_particles, const std::vector<cv::Vec3d>& r_particles, pfilter::MapReader& r_map_reader)
{
    cv::Mat free_map = r_map_reader.getFreeMap();
    cv::Mat image, image_color;
    int arrow_length = 4;
    free_map.convertTo(image, CV_8UC3, 255);
    cv::cvtColor(image, image_color, cv::COLOR_GRAY2BGR);
    for(int i = 0; i < num_particles; i++)
    {
        cv::Point2d point1, point2;
        point1.x = r_particles[i][0]/r_map_reader.resolution; point1.y = r_particles[i][1]/r_map_reader.resolution;
        double theta1 = r_particles[i][1];
        point2.x = std::round(point1.x + arrow_length*cos(theta1));
        point2.y = std::round(point1.y + arrow_length*sin(theta1));

        cv::circle(image_color, point1, 2, cv::Scalar(255, 0, 0), -1);
        cv::arrowedLine(image_color, point1, point2, cv::Scalar(0, 255, 0));
    }
    return image_color;
}

int main(int argc, char *argv[])
{
    // Read the filenames from the command line
    if(argc < 3)
    {
        std::cerr << " Usage: particle-filter <path-to-map> <path-to-log-file>" << std::endl;
        return 1;
    }
    std::string map_filename(argv[1]);
    std::string log_filename(argv[2]);

    std::fstream log_file;
    log_file.open(log_filename);

    if(!log_file.is_open())
    {
        std::cerr << "Error: Unable to read log file" << std::endl;
        return 2;
    }

    bool visualize = true;
    int num_particles = 5000;

    pfilter::MapReader map_reader(map_filename);
    pfilter::MotionModel motion_model(map_reader.resolution);

    std::vector<cv::Vec3d> particles;
    std::vector<double> weights;

    initParticles(num_particles, map_reader, particles, weights);

    if(visualize)
    {
        cv::Mat image = plotParticles(num_particles, particles, map_reader);
        cv::namedWindow("Map", CV_WINDOW_NORMAL);
        cv::imshow("Map", image);
        cv::waitKey(0);
    }

    std::string line;
    while(std::getline(log_file, line))
    {
        std::stringstream line_stream(line);
        char measure_type; double val;
        std::vector<double> measure_vals;

        line_stream >> measure_type;
        while(line_stream >> val)
            measure_vals.push_back(val);

    }

    return 0;
}
