#include "image_hint_transformer.h"
#include "lms/imaging_detection/line.h"
#include "lms/imaging_detection/splitted_line.h"

#include "lms/imaging/warp.h"
#include "lms/math/vertex.h"

bool ImageHintTransformer::initialize() {
    hintContainer = datamanager()->
            readChannel<lms::imaging::find::HintContainer>(this,"HINTS");

    environment = datamanager()->writeChannel<Environment>(this, "ENVIRONMENT");

    return true;
}

bool ImageHintTransformer::deinitialize() {
    return true;
}

bool ImageHintTransformer::cycle() {
    environment->lanes.clear();

    //TODO Just for testing, that has to be changed so it can be defined via config
    for(const lms::imaging::find::ImageHintBase *hint:hintContainer->hints){
        logger.error("HINT_NAME: ") <<hint << " " <<hint->name <<" pointCount "<<(static_cast<const lms::imaging::find::ImageHint<lms::imaging::find::Line>*>(hint))->imageObject.points().size();

        if(hint->name == "MIDDLE_LANE"){
            continue; //That continue does some real magic -.-
        }
        Environment::RoadLane lane;
        convertLane(hint,lane);
        if(hint->name == "RIGHT_LANE"){
            lane.type(Environment::RoadLaneType::RIGHT);

        }else if(hint->name == "LEFT_LANE"){
            lane.type(Environment::RoadLaneType::LEFT);

        }else if(hint->name == "MIDDLE_LANE"){
            lane.type(Environment::RoadLaneType::MIDDLE);
        }else{
            logger.error("cycle")<<"Convert lane with no type: " << hint->name;
        }
        environment->lanes.push_back(lane);
    }
    return true;
}

void ImageHintTransformer::convertLane(const lms::imaging::find::ImageHintBase *hint, Environment::RoadLane &lane){
    convertLine(static_cast<const lms::imaging::find::ImageHint<lms::imaging::find::Line>*>(hint)->imageObject,lane);
}

void ImageHintTransformer::convertLine(const lms::imaging::find::Line &line,Environment::RoadLane &lane){
    logger.info("HIER!");
    for(const lms::imaging::find::LinePoint &linePoint : line.points()) {
        logger.info("DA!");
        //lms::math::vertex2i in(linePoint.low_high.x, linePoint.low_high.y);
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
