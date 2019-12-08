
add_library(vsnakes include/vsnakes/vsnakes.h
                    src/vsnakes.cpp
                    include/vsnakes/detail/optimize_anchors.h
                    include/vsnakes/detail/optimize_anchors.hpp
                    include/vsnakes/utils.h
                    src/utils.cpp)

target_link_libraries(vsnakes PUBLIC eigen::eigen)
generate_export_header(vsnakes)

target_include_directories(vsnakes PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> "${CMAKE_CURRENT_BINARY_DIR}")

install(TARGETS vsnakes
        DESTINATION ${CMAKE_INSTALL_BINDIRs})

add_library(vsnakes::vsnakes ALIAS vsnakes)
