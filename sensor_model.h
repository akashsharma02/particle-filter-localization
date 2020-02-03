#ifndef SENSOR_MODEL_H
#define SENSOR_MODEL_H

#include "map_reader.h"

namespace pfilter
{
    class SensorModel
    {
    private:


    public:
        SensorModel(pfilter::MapReader& r_map_reader);
        virtual ~SensorModel();
    };
} /* namespace pfilter */
#endif /* SENSOR_MODEL_H */
