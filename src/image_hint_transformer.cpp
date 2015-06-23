#include "image_hint_transformer.h"
#include "lms/imaging_detection/line.h"
#include "lms/imaging_detection/splitted_line.h"
#include "lms/imaging_detection/point_line.h"

#include "lms/imaging/warp.h"
#include "lms/math/vertex.h"
#include "street_environment/obstacle.h"

bool ImageHintTransformer::initialize() {
    hintContainer = datamanager()->
            readChannel<lms::imaging::find::HintContainer>(this,"HINTS");

    environment = datamanager()->writeChannel<street_environment::Environment>(this, "ENVIRONMENT");

    middleEnv = datamanager()->writeChannel<street_environment::Environment>(this,"ENV_MID");
    return true;
}

bool ImageHintTransformer::deinitialize() {
    return true;
}

bool ImageHintTransformer::cycle() {
    environment->objects.clear();
    //TODO Just for testing, that has to be changed so it can be defined via config
    for(const lms::imaging::find::ImageHintBase *hint:hintContainer->hints){
        if(hint->name.find("LANE") != std::string::npos){
            std::shared_ptr<street_environment::RoadLane> lane(new street_environment::RoadLane());
            if(hint->name == "RIGHT_LANE"){
                lane->type(street_environment::RoadLaneType::RIGHT);
                logger.debug("cycle")<<"transform right lane";
            }else if(hint->name == "LEFT_LANE"){
                lane->type(street_environment::RoadLaneType::LEFT);
                logger.debug("cycle")<<"transform left lane";
            }else if(hint->name == "MIDDLE_LANE"){
                lane->type(street_environment::RoadLaneType::MIDDLE);
                logger.debug("cycle")<<"transform middle lane";
            }else{
                logger.warn("cycle")<<"Convert lane with no type: " << hint->name;
                continue;
            }
            if(lane->points().size() == 0){
                logger.info("Converting lane with no points!");
            }
            lane->name(hint->name);
            convertLane(hint,*lane);
            environment->objects.push_back(lane);
        }else if(hint->name.find("OBSTACLE") != std::string::npos){
            logger.debug("CONVERTING OBSTACLE!");
            int minObstaclePoints = 2;
            const lms::imaging::find::Line &line = static_cast<const lms::imaging::find::ImageHint<lms::imaging::find::Line>*>(hint)->imageObject;
            if((int)line.points().size() < minObstaclePoints){
                //Hint may or may not be valid, not enough points available!
                logger.debug("cycle")<<"Obstacle has not enough points: "<< line.points().size();
                continue;
            }

            if(middleEnv->objects.size() != 1){
                logger.error("createHintsFromMiddleLane")<<"no valid evironment for middle-lane";
                return true;
            }
            const street_environment::RoadLane &middle = middleEnv->objects[0]->getAsReference<const street_environment::RoadLane>();
            if(middle.type() != street_environment::RoadLaneType::MIDDLE){
                logger.error("createHintsFromMiddleLane") << "middle is no middle lane!";
                return true;
            }


            lms::math::vertex2f pos(0,0);
            lms::math::vertex2f tmp;
            for(const lms::imaging::find::LinePoint &lp:line.points()){
                lms::math::vertex2i v(lp.low_high.x,lp.low_high.y);
                lms::imaging::C2V(&v,&tmp);
                pos += tmp;
            }
            pos /= (float)line.points().size();

            std::shared_ptr<street_environment::Obstacle> obstacle(new street_environment::Obstacle());
            //TODO
            obstacle->updatePosition(pos);
            obstacle->name(hint->name);
            logger.debug("POS-DANACH") << obstacle->m_tmpPosition.x << " " <<obstacle->m_tmpPosition.y;
            environment->objects.push_back(obstacle);
        }else if(hint->name.find("CROSSING_LINE")){
            const lms::imaging::find::LineBase *line;
            if(hint->getHintType() == lms::imaging::find::PointLine::TYPE){
                line = &static_cast<const lms::imaging::find::ImageHint<lms::imaging::find::PointLine>*>(hint)->imageObject;
            }else if(hint->getHintType() == lms::imaging::find::Line::TYPE){
                line = &static_cast<const lms::imaging::find::ImageHint<lms::imaging::find::Line>*>(hint)->imageObject;
            }else{
                logger.error("cycle") << "Invalid CROSSING_LINE type given: "<< hint->getHintType();
                continue;
            }

            if(line->points().size() > 2){
                logger.info("cycle") << "Found Crossing_Line!";
            }
        }
    }
    return true;
}

void ImageHintTransformer::convertLane(const lms::imaging::find::ImageHintBase *hint, street_environment::RoadLane &lane){
    if(hint->getHintType() == lms::imaging::find::PointLine::TYPE){
        convertLine(static_cast<const lms::imaging::find::ImageHint<lms::imaging::find::PointLine>*>(hint)->imageObject,lane);
    }else if(hint->getHintType() == lms::imaging::find::Line::TYPE){
        convertLine(static_cast<const lms::imaging::find::ImageHint<lms::imaging::find::Line>*>(hint)->imageObject,lane);
    }
}

void ImageHintTransformer::convertLine(const lms::imaging::find::LineBase &line,street_environment::RoadLane &lane){
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
