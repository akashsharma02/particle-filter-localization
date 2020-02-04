#include "map_reader.h"

#include <fstream>

namespace pfilter
{
    MapReader::MapReader(std::string filename)
        : size_x(0)
        , size_y(0)
        , resolution(0)
    {

        std::ifstream map_file;
        std::string line;

        map_file.open(filename);

        if(map_file.is_open())
        {
            // Read the header information
            for(unsigned int i = 0; i < 7; i++)
            {
                std::getline(map_file, line);
                if(line.length() <= 10)
                    continue;

                // TODO: Correctly tokenize the strings to read the values
                if(line.compare(0, 38, "robot_specifications->global_mapsize_x") == 0)
                {
                    size_x = std::stoi(line.substr(line.length()-4, line.length()));
                }
                if(line.compare(0, 38, "robot_specifications->global_mapsize_y") == 0)
                {
                    size_y = std::stoi(line.substr(line.length()-4, line.length()));
                }
                if(line.compare(0, 32, "robot_specifications->resolution") == 0)
                {
                    resolution = std::stoi(line.substr(line.length()-3, line.length()));
                }
            }

            std::cout << "World size: (" << size_y << ", " << size_x << ") Resolution: " << resolution << " cm" << std::endl;
            cv::Mat occupancy = cv::Mat::zeros(size_x/resolution, size_y/resolution, CV_64F);
            cv::Mat free = cv::Mat::zeros(size_x/resolution, size_y/resolution, CV_64F);
            occupancy_map = cv::Mat::zeros(size_x/resolution, size_y/resolution, CV_64F);
            free_map      = cv::Mat::zeros(size_y/resolution, size_x/resolution, CV_64F);

            int count = 0; double val;
            while(map_file >> val)
            {
                int temprow = count / (size_x/resolution);
                int tempcol = count % (size_x/resolution);
                if(val < 0.0)
                {
                    occupancy.at<double>(temprow, tempcol) = -1.0;
                    free.at<double>(temprow, tempcol) = -1.0;
                }
                else if(val > 0.0)
                {
                    occupancy.at<double>(temprow, tempcol) = 1-val;
                    free.at<double>(temprow, tempcol) = val;
                }
                count++;
            }
            cv::flip(occupancy, occupancy_map, 0);
            cv::flip(free, free_map, 0);
            std::cout << "Finished reading map file: " << filename << std::endl;
        }
    }

    MapReader::~MapReader()
    {
        //TODO: Delete the memory
    }

    void MapReader::display(void)
    {
        cv::namedWindow("Map", CV_WINDOW_NORMAL);
        cv::imshow("Map", free_map);
        cv::waitKey(0);
    }

    cv::Mat& MapReader::getOccupancyMap()
    {
        return occupancy_map;
    }

    cv::Mat& MapReader::getFreeMap()
    {
        return free_map;
    }

} /* pfilter */

