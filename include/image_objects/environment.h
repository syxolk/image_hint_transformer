#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <vector>
#include <lms/imaging/warp.h>
#include "lms/math/vertex.h"

class Environment {
public:
    enum class RoadLaneType {
        LEFT, MIDDLE, RIGHT
    };

    struct RoadLane {
        RoadLaneType type;
        std::vector<lms::math::vertex2f> points;
    };
    std::vector<RoadLane> lanes;
};

#endif /* ENVIRONMENT_H */
