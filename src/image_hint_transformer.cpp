#include "image_hint_transformer.h"
#include "lms/imaging/find/line.h"
#include "lms/imaging/warp.h"

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
    environment->lanes.empty();

    const lms::imaging::find::ImageHintBase *hint = hintContainer->getByName("LEFT_LANE");

    Environment::RoadLane lane;
    lms::imaging::find::ImageHint<lms::imaging::find::Line> *line =
            (lms::imaging::find::ImageHint<lms::imaging::find::Line> *)hint;

    lane.type = Environment::RoadLaneType::LEFT;

    for(const lms::imaging::find::LinePoint &linePoint : line->imageObject.points()) {
        lms::imaging::vertex2i in(linePoint.low_high.x, linePoint.low_high.y);
        lms::imaging::vertex2f out;
        bool success = lms::imaging::C2V(&in, &out);

        if(success) {
            lane.points.push_back(lms::imaging::vertex2f(out[0], out[1]));
        }
    }

    environment->lanes.push_back(lane);

    return true;
}
