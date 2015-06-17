#ifndef IMAGE_HINT_TRANSFORMER_H
#define IMAGE_HINT_TRANSFORMER_H

#include "lms/module.h"
#include "lms/datamanager.h"
#include "lms/imaging_detection/image_hint.h"
#include "street_environment/road.h"
#include "lms/imaging_detection/line.h"

class ImageHintTransformer : public lms::Module {
public:
    bool initialize() override;
    bool deinitialize() override;
    bool cycle() override;
private:
    const lms::imaging::find::HintContainer *hintContainer;
    street_environment::Environment *environment;
    const street_environment::Environment *middleEnv;

    void convertLane(const lms::imaging::find::ImageHintBase *hint, street_environment::RoadLane &lane);
    void convertLine(const lms::imaging::find::LineBase &line,street_environment::RoadLane &lane);

};

#endif /* IMAGE_HINT_TRANSFORMER_H */
