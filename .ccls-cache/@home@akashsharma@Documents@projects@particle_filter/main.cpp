#include <iostream>
#include <fstream>
#include <random>
#include <opencv2/opencv.hpp>

#include "map_reader.hpp"
#include "motion_model.hpp"
#include "sensor_model.hpp"

void initParticles(int num_particles, pfilter::MapReader& r_map_reader, std::vector<cv::Vec3d>& r_particles, std::vector<double>& r_weights)
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
    cv::Mat image, image_color, image_color_flipped;
    int arrow_length = 6;
    free_map.convertTo(image, CV_8UC3, 255);
    cv::cvtColor(image, image_color, cv::COLOR_GRAY2BGR);
    cv::circle(image_color, cv::Point2i(0, 0), 10, cv::Scalar(0, 255, 0), -1);
    for(int i = 0; i < num_particles; i++)
    {
        cv::Point2i point1, point2;
        point1.x = r_particles[i][0]/r_map_reader.resolution; point1.y = r_particles[i][1]/r_map_reader.resolution;
        double theta1 = r_particles[i][2];
        point2.x = std::round(point1.x + arrow_length*cos(theta1));
        point2.y = std::round(point1.y + arrow_length*sin(theta1));

        cv::circle(image_color, point1, 3, cv::Scalar(255, 0, 0), -1);
        cv::arrowedLine(image_color, point1, point2, cv::Scalar(0, 255, 0));
    }
    cv::flip(image_color, image_color_flipped, 0);
    return image_color_flipped;
}

void resample(int num_particles, pfilter::MapReader& r_map_reader, std::vector<cv::Vec3d>& r_particles, std::vector<double>& r_weights)
{
    std::vector<cv::Vec3d> new_particles;
    std::vector<double> new_weights;
    std::default_random_engine generator;
    std::uniform_real_distribution<> r(0, 1.0);
    double random_val = r(generator);
    double offset = r_weights[0];
    int i = 0;
    for(unsigned int m = 1; m <= num_particles; m++)
    {
        double U = (random_val + (m-1))/num_particles;
        while(U > offset)
        {
            i++;
            offset = offset + r_weights[i];
        }
        new_particles.push_back(r_particles[i]);
        new_weights.push_back(r_weights[i]);
    }

    r_particles.assign(new_particles.begin(), new_particles.end());
    r_weights.assign(new_weights.begin(), new_weights.end());
}

int main(int argc, char *argv[])
{
    // Read the filenames from the command line and check for issues
    if(argc < 3)
    {
        std::cerr << " Usage: particle-filter <path-to-map> <path-to-log-file>" << std::endl;
        return 1;
    }
    std::string map_filename(argv[1]);
    std::string log_filename(argv[2]);

    pfilter::MapReader map_reader(map_filename);
    if(!map_reader.getMapState())
        return 2;

    std::fstream log_file;
    log_file.open(log_filename);

    if(!log_file.is_open())
    {
        std::cerr << "Error: Unable to read log file!" << std::endl;
        return 3;
    }
    //Read the log file number to initialize the max range of sensor
    std::string logfilenum_str = log_filename.substr(log_filename.find_last_of(".")-1, 1);
    int logfile_num = std::stoi(logfilenum_str);
    if(logfile_num < 1 || logfile_num > 5)
    {
        std::cerr << "Error: Unknown log file, LogFile format robotdata<number>.log" << std::endl;
        return 4;
    }

    pfilter::MotionModel motion_model(map_reader.resolution);
    pfilter::SensorModel sensor_model(map_reader, logfile_num-1);

    bool visualize = true;
    bool test_raycast = false;
    const int max_particles = 10000;
    int num_particles = max_particles;
    float particle_scaling_factor = 1.225;
    double reinitialization_threshold = 1.0;

    // Initialize the particles
    std::vector<cv::Vec3d> particles;
    std::vector<double> weights;
    initParticles(num_particles, map_reader, particles, weights);

    if(visualize)
    {
        cv::Mat image = plotParticles(num_particles, particles, map_reader);
        cv::namedWindow("Map", CV_WINDOW_NORMAL);
        cv::imshow("Map", image);
        std::cout << "Press a key to start" << std::endl;
        cv::waitKey(0);
    }

    std::string line;
    int line_count = 1;
    bool first_time = true;
    cv::Vec3d u_t0;
    while(std::getline(log_file, line))
    {
        if(line.size() == 0)
        {
            std::cerr << "Log file error: discontinuous. " << std::endl;
            return 5;
        }
        std::stringstream line_stream(line);
        char measure_type; double val;
        std::vector<double> measure_vals;

        line_stream >> measure_type;
        while(line_stream >> val)
            measure_vals.push_back(val);


        cv::Vec3d odometry_robot;
        odometry_robot[0] = measure_vals[0];
        odometry_robot[1] = measure_vals[1];
        odometry_robot[2] = measure_vals[2];

        double time_stamp = measure_vals[measure_vals.size()-1];

        cv::Vec3d odometry_laser;
        std::vector<double> ranges;
        if(measure_type == 'L')
        {
            odometry_laser[0] = measure_vals[3];
            odometry_laser[1] = measure_vals[4];
            odometry_laser[2] = measure_vals[5];
            std::vector<double>::iterator it(measure_vals.begin()+6);
            ranges.assign(it, measure_vals.end()-1);
        }

        std::cout << "Processing time step: " << line_count << " at time: " << time_stamp << std::endl;
        line_count++;

        if(first_time)
        {
            u_t0 = odometry_robot;
            first_time = false;
            continue;
        }

        cv::Vec3d u_t1 = odometry_robot;
        std::vector<cv::Vec3d> new_particles(num_particles);
        std::vector<double> new_weights(num_particles, 0);

        bool is_moving = motion_model.isMoving(u_t0, u_t1);
        if(!is_moving)
            continue;

        //TODO: Check whether this is necessary
        if(measure_type == 'O')
            continue;

        // For each particle
        double weight_norm = 0;
        double max_weight = 0;
        double min_weight = std::numeric_limits<double>::infinity();
#pragma omp parallel
        {
#pragma omp for
        for(unsigned int m = 0; m < num_particles; m++)
        {
            cv::Vec3d x_t0 = particles[m];
            double weight  = weights[m];
            cv::Vec3d x_t1 = motion_model.update(u_t0, u_t1, x_t0);
            if(measure_type == 'L')
            {
                assert(ranges.size() != 0);
                double w_t = sensor_model.beamRangeFinderModel(ranges, x_t1, test_raycast);
                if(w_t > max_weight)
                    max_weight = w_t;
                if(w_t < min_weight)
                    min_weight = w_t;
                weight_norm = weight_norm + w_t;
                new_weights[m] = w_t;
            }
            else
            {
                new_weights[m] = weight;
            }
            new_particles[m] = x_t1;
        }
        } // end of parallel block
        //normalize the weights
        assert(weight_norm != 0);

        for(unsigned int m = 0; m < num_particles; m++)
            new_weights[m] /= weight_norm;

        //Handle kidnapped robot problem
        if(max_weight/weight_norm <= (reinitialization_threshold * 1/num_particles))
        {
            new_particles.clear(); new_weights.clear();
            num_particles = max_particles;
            initParticles(num_particles, map_reader, new_particles, new_weights);
        }
        else
        {
            //Adaptively resample the number of particles
            float difference = max_weight - min_weight;
            float sum = max_weight + min_weight;
            if(difference != 0.0 && sum/difference > particle_scaling_factor)
                num_particles = num_particles - num_particles/500;
            else
                num_particles = num_particles + num_particles/500;
            resample(num_particles, map_reader, new_particles, new_weights);
        }

        particles.assign(new_particles.begin(), new_particles.end());
        weights.assign(new_weights.begin(), new_weights.end());
        u_t0 = u_t1;

        if(visualize)
        {
            cv::Mat image = plotParticles(num_particles, particles, map_reader);
            cv::imshow("Map", image);
            char c = static_cast<char>(cv::waitKey(25));
            if(c == 27)
                break;
        }
    }
    return 0;
}
