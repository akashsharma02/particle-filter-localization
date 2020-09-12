/******************************************************************************
* File:             sampling.hpp
*
* Author:           Akash Sharma
* Created:          09/11/20
* Description:      Functions related to particle sampling
*****************************************************************************/
#ifndef PARTICLES_HPP
#define PARTICLES_HPP

void initParticles(unsigned int num_particles,
                   pfilter::MapReader& r_map_reader,
                   std::vector<cv::Vec3d>& particles,
                   std::vector<double>& weights)
{
    cv::Mat occupancy_map = r_map_reader.getOccupancyMap();
    std::random_device rd;
    std::mt19937 generator(rd());
    /* std::default_random_engine generator; */
    std::uniform_int_distribution<> x_val_distribution(3000, 7000);
    std::uniform_int_distribution<> y_val_distribution(0, 7500);
    std::uniform_real_distribution<> theta_distribution(-3.14, 3.14);
    std::uniform_real_distribution<> u_distribution(0, 1);

    /* std::vector<cv::Vec3d> particles(num_particles); */
    std::vector<double> u_vals(static_cast<size_t>(num_particles * 10));
    weights.resize(static_cast<size_t>(num_particles), 1 / num_particles);

    // Rejection sampling until num_particles reached
    for (unsigned int i = 0; particles.size() < num_particles; i++)
    {
        cv::Vec3d particle;
        particle[0]  = x_val_distribution(generator);
        particle[1]  = y_val_distribution(generator);
        particle[2]  = theta_distribution(generator);
        u_vals.at(i) = u_distribution(generator);

        cv::Point2i particle_point(static_cast<int>(particle[0] / r_map_reader.resolution_),
                                   static_cast<int>(particle[1] / r_map_reader.resolution_));
        if (u_vals.at(i) > occupancy_map.at<double>(particle_point) && occupancy_map.at<double>(particle_point) >= 0.0)
            particles.push_back(particle);
    }
}

cv::Mat plotParticles(unsigned int num_particles, const std::vector<cv::Vec3d>& particles, pfilter::MapReader& r_map_reader)
{
    cv::Mat free_map = r_map_reader.getFreeMap();
    cv::Mat image, image_color, image_color_flipped;
    int arrow_length = 6;
    free_map.convertTo(image, CV_8UC3, 255);
    cv::cvtColor(image, image_color, cv::COLOR_GRAY2BGR);
    cv::circle(image_color, cv::Point2i(0, 0), 10, cv::Scalar(0, 255, 0), -1);
    for (unsigned int i = 0; i < num_particles; i++)
    {
        cv::Point2i point1, point2;
        point1.x      = static_cast<int>(particles[i][0] / r_map_reader.resolution_);
        point1.y      = static_cast<int>(particles[i][1] / r_map_reader.resolution_);
        double theta1 = particles[i][2];
        point2.x      = static_cast<int>(std::round(point1.x + arrow_length * cos(theta1)));
        point2.y      = static_cast<int>(std::round(point1.y + arrow_length * sin(theta1)));

        cv::circle(image_color, point1, 3, cv::Scalar(255, 0, 0), -1);
        cv::arrowedLine(image_color, point1, point2, cv::Scalar(0, 255, 0));
    }
    cv::flip(image_color, image_color_flipped, 0);
    return image_color_flipped;
}

void resample(unsigned int num_particles, std::vector<cv::Vec3d>& particles, std::vector<double>& weights)
{
    std::vector<cv::Vec3d> new_particles;
    std::vector<double> new_weights;
    std::default_random_engine generator;
    std::uniform_real_distribution<> r(0, 1.0);
    double random_val = r(generator);
    double offset     = weights[0];
    unsigned int i    = 0;
    for (unsigned int m = 1; m <= num_particles; m++)
    {
        double U = (random_val + (m - 1)) / num_particles;
        while (U > offset)
        {
            i++;
            offset = offset + weights[i];
        }
        new_particles.push_back(particles[i]);
        new_weights.push_back(weights[i]);
    }

    particles.assign(new_particles.begin(), new_particles.end());
    weights.assign(new_weights.begin(), new_weights.end());
}
#endif /* ifndef PARTICLE_FUNCS_HPP */
