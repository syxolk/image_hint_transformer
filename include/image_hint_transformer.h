#ifndef IMAGE_HINT_TRANSFORMER_H
#define IMAGE_HINT_TRANSFORMER_H

#include "lms/module.h"
#include "lms/datamanager.h"
#include "lms/imaging/find/image_hint.h"
#include "image_objects/environment.h"

class ImageHintTransformer : public lms::Module {
public:
    bool initialize() override;
    bool deinitialize() override;
    bool cycle() override;
private:
    const lms::imaging::find::HintContainer *hintContainer;
    Environment *environment;
    void convertLane(const lms::imaging::find::ImageHintBase *hint, Environment::RoadLane &lane);
};

#endif /* IMAGE_HINT_TRANSFORMER_H */
