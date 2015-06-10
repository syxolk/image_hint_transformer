#include "image_hint_transformer.h"
#include "lms/imaging_detection/line.h"
#include "lms/imaging_detection/splitted_line.h"

#include "lms/imaging/warp.h"
#include "lms/math/vertex.h"

bool ImageHintTransformer::initialize() {
    hintContainer = datamanager()->
            readChannel<lms::imaging::find::HintContainer>(this,"HINTS");

    environment = datamanager()->writeChannel<street_environment::Environment>(this, "ENVIRONMENT");

    return true;
}

bool ImageHintTransformer::deinitialize() {
    return true;
}

bool ImageHintTransformer::cycle() {
    environment->objects.clear();
    //TODO Just for testing, that has to be changed so it can be defined via config
    for(const lms::imaging::find::ImageHintBase *hint:hintContainer->hints){
        std::shared_ptr<street_environment::RoadLane> lane(new street_environment::RoadLane());
        convertLane(hint,*lane);
        if(hint->name == "RIGHT_LANE"){
            lane->type(street_environment::RoadLaneType::RIGHT);

        }else if(hint->name == "LEFT_LANE"){
            lane->type(street_environment::RoadLaneType::LEFT);

        }else if(hint->name == "MIDDLE_LANE"){
            lane->type(street_environment::RoadLaneType::MIDDLE);
        }else{
            logger.error("cycle")<<"Convert lane with no type: " << hint->name;
        }
        environment->objects.push_back(lane);
    }
    return true;
}

void ImageHintTransformer::convertLane(const lms::imaging::find::ImageHintBase *hint, street_environment::RoadLane &lane){
    convertLine(static_cast<const lms::imaging::find::ImageHint<lms::imaging::find::Line>*>(hint)->imageObject,lane);
}

void ImageHintTransformer::convertLine(const lms::imaging::find::Line &line,street_environment::RoadLane &lane){
    for(const lms::imaging::find::LinePoint &linePoint : line.points()) {
        lms::math::vertex2f out;
        lms::math::vertex2i vi;
        vi.x = linePoint.low_high.x;
        vi.y = linePoint.low_high.y;
        bool success = lms::imaging::C2V(&vi, &out);

        if(success) {
            lane.points().push_back(out);
        }
    }
}

void convertMiddleLine(){

}
