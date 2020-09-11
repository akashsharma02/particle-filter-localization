#ifndef MAP_READER_HPP
#define MAP_READER_HPP

#include <opencv2/opencv.hpp>
#include <string>

namespace pfilter
{
    class MapReader
    {
       public:
        MapReader(std::string filename);
        virtual ~MapReader();

        void display(void);

        cv::Mat& getOccupancyMap(void);
        cv::Mat& getFreeMap(void);
        inline bool getMapState(void) { return map_state_; }

        int size_x_, size_y_;
        int resolution_;

       private:
        std::string filename_;
        cv::Mat occupancy_map_, free_map_;
        bool map_state_;
    };
}  // namespace pfilter

#endif /* MAP_READER_HPP */
