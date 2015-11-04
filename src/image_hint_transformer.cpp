#include "image_hint_transformer.h"
#include "lms/imaging_detection/line.h"
#include "lms/imaging_detection/splitted_line.h"
#include "lms/imaging_detection/point_line.h"
#include "lms/imaging_detection/street_crossing.h"
#include "lms/imaging_detection/street_obstacle.h"


#include "lms/imaging/warp.h"
#include "lms/math/vertex.h"
#include "street_environment/obstacle.h"
#include "street_environment/crossing.h"

bool ImageHintTransformer::initialize() {
    hintContainer = datamanager()->
            readChannel<lms::imaging::detection::HintContainer>(this,"HINTS");

    environment = datamanager()->writeChannel<street_environment::EnvironmentObjects>(this, "ENVIRONMENT");
    return true;
}

bool ImageHintTransformer::deinitialize() {
    return true;
}

bool ImageHintTransformer::cycle() {
    environment->objects.clear();
    //TODO remove name checking!
    //TODO Just for testing, that has to be changed so it can be defined via config
    for(const lms::imaging::detection::ImageHintBase *hint:hintContainer->hints){
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
            if(hint->getHintType() == lms::imaging::detection::Line::TYPE){
                logger.debug("CONVERTING OBSTACLE!");
                int minObstaclePoints = 2;
                const lms::imaging::detection::Line &line = static_cast<const lms::imaging::detection::ImageHint<lms::imaging::detection::Line>*>(hint)->imageObject;
                if((int)line.points().size() < minObstaclePoints){
                    //Hint may or may not be valid, not enough points available!
                    logger.debug("cycle")<<"Obstacle has not enough points: "<< line.points().size();
                    continue;
                }

                lms::math::vertex2f pos(0,0);
                lms::math::vertex2f tmp;
                for(const lms::imaging::detection::LinePoint &lp:line.points()){
                    lms::math::vertex2i v(lp.low_high.x,lp.low_high.y);
                    lms::imaging::C2V(&v,&tmp);
                    pos += tmp;
                }
                pos /= (float)line.points().size();

                std::shared_ptr<street_environment::Obstacle> obstacle(new street_environment::Obstacle());

                obstacle->updatePosition(pos);
                obstacle->name(hint->name);
                environment->objects.push_back(obstacle);
            }else if(hint->getHintType() == lms::imaging::detection::StreetObstacle::TYPE){
                const lms::imaging::detection::StreetObstacle &obs = static_cast<const lms::imaging::detection::ImageHint<lms::imaging::detection::StreetObstacle>*>(hint)->imageObject;

                lms::math::vertex2f pos(0,0);
                lms::math::vertex2f tmp;
                for(const lms::imaging::detection::LinePoint &lp:obs.edgeLine.points()){
                    lms::math::vertex2i v(lp.low_high.x,lp.low_high.y);
                    lms::imaging::C2V(&v,&tmp);
                    pos += tmp;
                }
                pos /= (float)obs.edgeLine.points().size();

                std::shared_ptr<street_environment::Obstacle> obstacle(new street_environment::Obstacle());

                obstacle->updatePosition(pos);
                obstacle->name(hint->name);
                environment->objects.push_back(obstacle);


            }
        }else if(hint->name.find("CROSSING")){
            const lms::imaging::detection::StreetCrossing *crossingImage;
            crossingImage = &static_cast<const lms::imaging::detection::ImageHint<lms::imaging::detection::StreetCrossing>*>(hint)->imageObject;

            lms::math::vertex2f out;
            lms::math::vertex2i vi;
            vi.x = crossingImage->x();
            vi.y = crossingImage->y();
            lms::imaging::C2V(&vi, &out);


            std::shared_ptr<street_environment::Crossing> crossing(new street_environment::Crossing());
            crossing->blocked(crossingImage->blocked);

            environment->objects.push_back(crossing);
        }
    }
    return true;
}

void ImageHintTransformer::convertLane(const lms::imaging::detection::ImageHintBase *hint, street_environment::RoadLane &lane){
    if(hint->getHintType() == lms::imaging::detection::PointLine::TYPE){
        convertLine(static_cast<const lms::imaging::detection::ImageHint<lms::imaging::detection::PointLine>*>(hint)->imageObject,lane);
    }else if(hint->getHintType() == lms::imaging::detection::Line::TYPE){
        convertLine(static_cast<const lms::imaging::detection::ImageHint<lms::imaging::detection::Line>*>(hint)->imageObject,lane);
    }
}

void ImageHintTransformer::convertLine(const lms::imaging::detection::LineBase &line,street_environment::RoadLane &lane){
    for(const lms::imaging::detection::LinePoint &linePoint : line.points()) {
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
