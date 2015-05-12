#ifndef IMAGE_HINT_TRANSFORMER_H
#define IMAGE_HINT_TRANSFORMER_H

#include "lms/module.h"
#include "lms/datamanager.h"
#include "lms/imaging/find/image_hint.h"
#include "environment.h"

class ImageHintTransformer : public lms::Module {
public:
    bool initialize() override;
    bool deinitialize() override;
    bool cycle() override;
private:
    const lms::imaging::find::HintContainer *hintContainer;
    Environment *environment;
};

#endif /* IMAGE_HINT_TRANSFORMER_H */
