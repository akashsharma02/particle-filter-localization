#ifndef MAP_READER_HPP
#define MAP_READER_HPP

#include <string>
#include <opencv2/opencv.hpp>

namespace pfilter
{
    class MapReader
    {
    private:
        cv::Mat occupancy_map, free_map;
        bool map_state;

    public:
        MapReader(std::string filename);
        virtual ~MapReader();

        void display(void);

        cv::Mat& getOccupancyMap(void);
        cv::Mat& getFreeMap(void);
        inline bool getMapState(void) { return map_state; }

        int size_x, size_y;
        int resolution;
    };
} /* pfilter */

#endif /* MAP_READER_HPP */
