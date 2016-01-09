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
#include "street_environment/start_line.h"

bool ImageHintTransformer::initialize() {
    hintContainer = readChannel<lms::imaging::detection::HintContainer>("HINTS");
    environment = writeChannel<street_environment::EnvironmentObjects>("ENVIRONMENT");
    return true;
}

bool ImageHintTransformer::deinitialize() {
    return true;
}

bool ImageHintTransformer::cycle() {
    environment->objects.clear();
    //TODO remove name checking!
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
        }else if(hint->getHintType() == lms::imaging::detection::StreetObstacle::TYPE){
                const lms::imaging::detection::StreetObstacle &obs = static_cast<const lms::imaging::detection::ImageHint<lms::imaging::detection::StreetObstacle>*>(hint)->imageObject;

                logger.debug(getName())<<"found obstacle pointCount: "<<&obs.results<<" "<<obs.results.size();
                for(const lms::imaging::detection::Line &line:obs.results){
                    logger.debug(getName())<<"found obstacle pointCount: "<<line.points().size();
                    //get the position
                    lms::math::vertex2f pos(0,0);
                    lms::math::vertex2f tmp;
                    for(const lms::imaging::detection::LinePoint &lp:line.points()){
                        lms::math::vertex2i v(lp.low_high.x,lp.low_high.y);
                        lms::imaging::C2V(&v,&tmp);
                        pos += tmp;
                    }
                    pos /= (float)line.points().size();
                    logger.debug("cycle")<<"adding obstacle at"<<pos.x << " "<<pos.y;
                    //get the viewDirection

                    lms::math::vertex2i i1(line.points()[0].low_high.x,line.points()[0].low_high.y);
                    lms::math::vertex2i i2(line.points()[line.points().size()-1].low_high.x,line.points()[line.points().size()-1].low_high.y);

                    lms::math::vertex2f pos1(0,0);
                    lms::math::vertex2f pos2(0,0);
                    lms::imaging::C2V(&i1,&pos1);
                    lms::imaging::C2V(&i2,&pos2);
                    lms::math::vertex2f viewDir = (pos1-pos2).rotateAntiClockwise90deg();
                    float obstWidth = pos1.distance(pos2);

                    std::shared_ptr<street_environment::Obstacle> obstacle(new street_environment::Obstacle());
                    obstacle->setTrust(0.1);//set the trust
                    obstacle->updatePosition(pos);
                    obstacle->name(hint->name);
                    obstacle->viewDirection(viewDir);
                    obstacle->width(obstWidth);
                    environment->objects.push_back(obstacle);
                }
        }else if(hint->getHintType() == lms::imaging::detection::StreetCrossing::TYPE){
            const lms::imaging::detection::StreetCrossing *crossingImage = &static_cast<const lms::imaging::detection::ImageHint<lms::imaging::detection::StreetCrossing>*>(hint)->imageObject;
            if(crossingImage->stopLine.points().size() < 2){
                continue;
            }
            lms::math::vertex2f out;
            lms::math::vertex2i vi;
            vi.x = crossingImage->x();
            vi.y = crossingImage->y();
            lms::imaging::C2V(&vi, &out);
            lms::math::vertex2i vi1 = static_cast<lms::math::vertex2i>(crossingImage->stopLine.points()[0].low_high);
            lms::math::vertex2i vi2= static_cast<lms::math::vertex2i>(crossingImage->stopLine.points()[crossingImage->stopLine.points().size()-1].low_high);
            lms::math::vertex2f out1;
            lms::math::vertex2f out2;
            lms::imaging::C2V(&vi1, &out1);
            lms::imaging::C2V(&vi2, &out2);
            if(crossingImage->foundStartLine){
                std::shared_ptr<street_environment::StartLine> startLine(new street_environment::StartLine());
                startLine->updatePosition(out);
                startLine->viewDirection((out1-out2).rotateAntiClockwise90deg());
                startLine->setTrust(0.5);
                environment->objects.push_back(startLine);
            }else if(crossingImage->foundCrossing){
                std::shared_ptr<street_environment::Crossing> crossing(new street_environment::Crossing());
                crossing->blocked(crossingImage->blocked);
                crossing->viewDirection((out1-out2).rotateAntiClockwise90deg());
                crossing->setTrust(0.5);
                crossing->updatePosition(out);
                environment->objects.push_back(crossing);
            }else{
                continue;
            }
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
