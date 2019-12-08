
add_executable(tests_unit_vsnakes tests_unit_vsnakes.cpp)
target_link_libraries(tests_unit_vsnakes vsnakes::vsnakes gtest::gtest)

install(TARGETS tests_unit_vsnakes
        DESTINATION ${CMAKE_INSTALL_BINDIRs})
