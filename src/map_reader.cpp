#include "map_reader.hpp"

#include <spdlog/spdlog.h>
#include <fstream>

namespace pfilter
{
    MapReader::MapReader(std::string filename)
        : size_x_(0), size_y_(0), resolution_(0), filename_(std::move(filename)), map_state_(true)
    {
        std::ifstream map_file;
        std::string line;

        map_file.open(filename_);

        if (map_file.is_open())
        {
            // Read the header information
            for (unsigned int i = 0; i < 7; i++)
            {
                std::getline(map_file, line);
                if (line.length() <= 10)
                    continue;

                // TODO: Correctly tokenize the strings to read the values
                if (line.compare(0, 38, "robot_specifications->global_mapsize_x") == 0)
                {
                    size_x_ = std::stoi(line.substr(line.length() - 4, line.length()));
                }
                if (line.compare(0, 38, "robot_specifications->global_mapsize_y") == 0)
                {
                    size_y_ = std::stoi(line.substr(line.length() - 4, line.length()));
                }
                if (line.compare(0, 32, "robot_specifications->resolution") == 0)
                {
                    resolution_ = std::stoi(line.substr(line.length() - 3, line.length()));
                }
            }

            spdlog::info("World size: ({}, {}) Resolution: {} cm", size_y_, size_x_, resolution_);

            cv::Mat occupancy = cv::Mat::zeros(size_x_ / resolution_, size_y_ / resolution_, CV_64F);
            cv::Mat free      = cv::Mat::zeros(size_x_ / resolution_, size_y_ / resolution_, CV_64F);
            occupancy_map_    = cv::Mat::zeros(size_x_ / resolution_, size_y_ / resolution_, CV_64F);
            free_map_         = cv::Mat::zeros(size_y_ / resolution_, size_x_ / resolution_, CV_64F);

            int count = 0;
            double val;
            while (map_file >> val)
            {
                int temprow = count / (size_x_ / resolution_);
                int tempcol = count % (size_x_ / resolution_);
                if (val < 0.0)
                {
                    occupancy.at<double>(temprow, tempcol) = -1.0;
                    free.at<double>(temprow, tempcol)      = -1.0;
                }
                else if (val > 0.0)
                {
                    occupancy.at<double>(temprow, tempcol) = 1 - val;
                    free.at<double>(temprow, tempcol)      = val;
                }
                count++;
            }
            cv::flip(occupancy, occupancy_map_, 0);
            cv::flip(free, free_map_, 0);
            spdlog::info("Finished reading map file: {}", filename);
        }
        else
        {
            spdlog::error("Unable to read map file: {}", filename);
            map_state_ = false;
        }
    }

    MapReader::~MapReader()
    {
        // TODO: Delete the memory
    }

    void MapReader::display(void)
    {
        cv::namedWindow("Map", cv::WINDOW_NORMAL);
        cv::imshow("Map", free_map_);
        cv::waitKey(0);
    }

    cv::Mat& MapReader::getOccupancyMap() { return occupancy_map_; }

    cv::Mat& MapReader::getFreeMap() { return free_map_; }

}  // namespace pfilter
