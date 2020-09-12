#include <spdlog/spdlog.h>
#include <CLI/CLI.hpp>
#include <opencv2/opencv.hpp>

#include <fstream>
#include <iostream>
#include <random>

#include "map_reader.hpp"
#include "motion_model.hpp"
#include "sensor_model.hpp"
#include "particles.hpp"

int main(int argc, char* argv[])
{
    // Read the filenames from the command line and check for issues
    CLI::App app{ "Particle filter" };

    std::string map_filename;
    app.add_option("path-to-map", map_filename, "Path to the map file")->required();

    std::string log_filename;
    app.add_option("path-to-log", log_filename, "Path to the log file")->required();

    bool test_raycast = false;
    app.add_flag("-t,--test-raycast", test_raycast, "Test raycasting functionality");

    bool visualize;
    app.add_flag("-v,--visualize", visualize, "Visualize the map");

    CLI11_PARSE(app, argc, argv);

    spdlog::info("Test raycast: {} Visualize: {}", test_raycast, visualize);
    pfilter::MapReader map_reader(map_filename);
    if (!map_reader.getMapState())
    {
        spdlog::error("Could not read map");
        return EXIT_FAILURE;
    }

    std::fstream log_file;
    log_file.open(log_filename);

    if (!log_file.is_open())
    {
        spdlog::error("Error: Unable to read log file!");
        return EXIT_FAILURE;
    }
    // Read the log file number to initialize the max range of sensor
    std::string logfilenum_str = log_filename.substr(log_filename.find_last_of(".") - 1, 1);
    int logfile_num            = std::stoi(logfilenum_str);
    if (logfile_num < 1 || logfile_num > 5)
    {
        spdlog::error("Error: Unknown log file {}, LogFile format robotdata<number>.log", logfilenum_str);
        return EXIT_FAILURE;
    }

    pfilter::MotionModel motion_model(map_reader.resolution_);
    pfilter::SensorModel sensor_model(map_reader, static_cast<size_t>(logfile_num - 1));

    constexpr unsigned int MAX_PARTICLES        = 10000;
    constexpr double particle_scaling_factor    = 1.225;
    constexpr double reinitialization_threshold = 1.0;
    constexpr double respawn_distance_threshold = 50.0;
    unsigned int num_particles                  = MAX_PARTICLES;

    // Initialize the particles
    std::vector<cv::Vec3d> particles;
    std::vector<double> weights;
    initParticles(num_particles, map_reader, particles, weights);

    if (visualize)
    {
        cv::Mat image = plotParticles(num_particles, particles, map_reader);
        cv::namedWindow("Map", cv::WINDOW_NORMAL);
        cv::imshow("Map", image);
        spdlog::info("Press a key to start");
        cv::waitKey(0);
    }

    std::string line;
    int line_count  = 1;
    bool first_time = true;
    cv::Vec3d u_t0;
    while (std::getline(log_file, line))
    {
        if (line.size() == 0)
        {
            std::cerr << "Log file error: discontinuous. " << std::endl;
            return 5;
        }
        std::stringstream line_stream(line);
        char measure_type;
        double val;
        std::vector<double> measure_vals;

        line_stream >> measure_type;
        while (line_stream >> val)
            measure_vals.push_back(val);

        cv::Vec3d odometry_robot;
        odometry_robot[0] = measure_vals[0];
        odometry_robot[1] = measure_vals[1];
        odometry_robot[2] = measure_vals[2];

        double time_stamp = measure_vals[measure_vals.size() - 1];

        cv::Vec3d odometry_laser;
        std::vector<double> ranges;
        if (measure_type == 'L')
        {
            odometry_laser[0] = measure_vals[3];
            odometry_laser[1] = measure_vals[4];
            odometry_laser[2] = measure_vals[5];
            std::vector<double>::iterator it(measure_vals.begin() + 6);
            ranges.assign(it, measure_vals.end() - 1);
        }

        spdlog::info("Processing time step: {} at time: {}", line_count, time_stamp);
        line_count++;

        if (first_time)
        {
            u_t0       = odometry_robot;
            first_time = false;
            continue;
        }

        cv::Vec3d u_t1 = odometry_robot;
        std::vector<cv::Vec3d> new_particles(num_particles);
        std::vector<double> new_weights(num_particles, 0);

        bool is_moving = motion_model.isMoving(u_t0, u_t1);
        if (!is_moving)
            continue;

        // TODO: Check whether this is necessary
        if (measure_type == 'O')
            continue;

        // For each particle
        double weight_norm = 0;
        double max_weight  = 0;
        double min_weight  = std::numeric_limits<double>::infinity();
#pragma omp parallel
        {
#pragma omp for
            for (unsigned int m = 0; m < num_particles; m++)
            {
                cv::Vec3d x_t0 = particles[m];
                double weight  = weights[m];
                cv::Vec3d x_t1 = motion_model.update(u_t0, u_t1, x_t0);
                if (measure_type == 'L')
                {
                    assert(ranges.size() != 0);
                    double w_t = sensor_model.beamRangeFinderModel(ranges, x_t1, test_raycast);
                    if (w_t > max_weight)
                        max_weight = w_t;
                    if (w_t < min_weight)
                        min_weight = w_t;
                    weight_norm    = weight_norm + w_t;
                    new_weights[m] = w_t;
                }
                else
                {
                    new_weights[m] = weight;
                }
                new_particles[m] = x_t1;
            }
        }  // end of parallel block
        // normalize the weights
        assert(weight_norm != 0);

        for (unsigned int m = 0; m < num_particles; m++)
            new_weights[m] /= weight_norm;

        // Handle kidnapped robot problem
        if (motion_model.distanceMoved() > respawn_distance_threshold ||
            max_weight / weight_norm <= (reinitialization_threshold * 1 / num_particles))
        {
            new_particles.clear();
            new_weights.clear();
            num_particles = MAX_PARTICLES;
            initParticles(num_particles, map_reader, new_particles, new_weights);
        }
        else
        {
            // Adaptively resample the number of particles
            double difference = max_weight - min_weight;
            double sum        = max_weight + min_weight;
            if (difference != 0.0 && sum / difference > particle_scaling_factor)
                num_particles = num_particles - num_particles / 500;
            else
                num_particles = num_particles + num_particles / 500;
            resample(num_particles, new_particles, new_weights);
        }

        particles.assign(new_particles.begin(), new_particles.end());
        weights.assign(new_weights.begin(), new_weights.end());
        u_t0 = u_t1;

        if (visualize)
        {
            cv::Mat image = plotParticles(num_particles, particles, map_reader);
            cv::imshow("Map", image);
            char c = static_cast<char>(cv::waitKey(25));
            if (c == 27)
                break;
        }
    }
    return 0;
}
