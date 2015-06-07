#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <vector>
#include <lms/imaging/warp.h>
#include "lms/math/vertex.h"
#include "lms/math/polyline.h"

class Environment {
public:
    enum class RoadLaneType {
        LEFT, MIDDLE, RIGHT
    };

    class RoadLane:public lms::math::polyLine2f{
        RoadLaneType m_type;


    public:

        std::vector<double> polarDarstellung; //TODO english name :)
        float polarPartLength;
        RoadLaneType type() const{
            return m_type;
        }

        void type(RoadLaneType type){
            m_type = type;
        }
    };
    std::vector<RoadLane> lanes;
};

#endif /* ENVIRONMENT_H */
