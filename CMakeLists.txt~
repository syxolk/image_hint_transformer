set ( SOURCES 
	"src/image_hint_transformer.cpp"
	"src/interface.cpp"
)

set (HEADERS
	"include/image_hint_transformer.h"
    "include/environment.h"
)

include_directories("include")

add_library (image_hint_transformer MODULE ${SOURCES} ${HEADERS})
target_link_libraries(image_hint_transformer PRIVATE lms imaging)
