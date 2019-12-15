
add_executable(optimize_anchors optimize_anchors.cpp)
target_link_libraries(optimize_anchors eigen::eigen vsnakes::vsnakes)

install(TARGETS optimize_anchors
        DESTINATION ${CMAKE_INSTALL_BINDIRs})
