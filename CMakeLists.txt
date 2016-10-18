set ( SOURCES
	"src/image_hint_transformer.cpp"
	"src/interface.cpp"
)

set (HEADERS
	"include/image_hint_transformer.h"
)

include_directories("include")

add_library (image_hint_transformer MODULE ${SOURCES} ${HEADERS})
target_link_libraries(image_hint_transformer PRIVATE lmscore lms_math lms_imaging lms_imaging_detection street_environment)
