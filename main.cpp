#include <iostream>
#include <opencv2/opencv.hpp>

#include "map_reader.h"
#include "motion_model.h"

int main(int argc, char *argv[])
{
    if(argc < 3)
    {
        std::cerr << argc << " Usage: particle-filter <path-to-map> <path-to-log-file>" << std::endl;
        return 1;
    }
    std::string map_filename(argv[1]);
    std::string log_filename(argv[2]);

    pfilter::MapReader map_reader(map_filename);
    pfilter::MotionModel motion_model(map_reader.resolution);
    map_reader.display();
    return 0;
}
