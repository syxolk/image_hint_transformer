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
    Environment::RoadLane lane;
    const lms::imaging::find::ImageHintBase *hint = hintContainer->getByName("LEFT_LANE");
    if(hint != nullptr){
    lane.type(Environment::RoadLaneType::LEFT);
    convertLane(hint,lane);
    environment->lanes.push_back(lane);
    }else{
        logger.warn() << "LEFT LANE IS NULL";
    }

    hint = hintContainer->getByName("RIGHT_LANE");
    if(hint != nullptr){
        lane.type(Environment::RoadLaneType::RIGHT);
        lane.points().clear();
        convertLane(hint,lane);
    environment->lanes.push_back(lane);
    }else{
        logger.warn() << "RIGHT LANE IS NULL";
    }

    hint = hintContainer->getByName("MIDDLE_LANE");
    if(hint != nullptr){
        lane.type(Environment::RoadLaneType::MIDDLE);
        lane.points().clear();
        convertMiddleLane(hint,lane);
        environment->lanes.push_back(lane);
    }

    return true;
}


void ImageHintTransformer::convertMiddleLane(const lms::imaging::find::ImageHintBase *hint, Environment::RoadLane &lane){
    lms::imaging::find::ImageHint<lms::imaging::find::SplittedLine> *line =
            (lms::imaging::find::ImageHint<lms::imaging::find::SplittedLine> *)hint;
    for(const lms::imaging::find::Line &l : line->imageObject.lines()){
        convertLine(l,lane);
    }
}

void ImageHintTransformer::convertLane(const lms::imaging::find::ImageHintBase *hint, Environment::RoadLane &lane){
    lms::imaging::find::ImageHint<lms::imaging::find::Line> *line =
            (lms::imaging::find::ImageHint<lms::imaging::find::Line> *)hint;
    convertLine(line->imageObject,lane);
}

void ImageHintTransformer::convertLine(const lms::imaging::find::Line &line,Environment::RoadLane &lane){
    for(const lms::imaging::find::LinePoint &linePoint : line.points()) {
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
