#ifndef MAP_READER_H
#define MAP_READER_H

#include <string>
#include <opencv2/opencv.hpp>

namespace pfilter
{
    class MapReader
    {
    private:
        cv::Mat occupancy_map, free_map;

    public:
        MapReader(std::string filename);
        virtual ~MapReader();

        void display(void);

        cv::Mat& getOccupancyMap(void);
        cv::Mat& getFreeMap(void);

        int size_x, size_y;
        int resolution;
    };
} /* pfilter */

#endif /* MAP_READER_H */
