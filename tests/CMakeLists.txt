add_executable(falconguide_core_tests core_tests.cpp)
target_link_libraries(falconguide_core_tests PRIVATE FalconGuide::core FalconGuide::estimation FalconGuide::io)
add_test(NAME falconguide_core_tests COMMAND falconguide_core_tests)

add_executable(falconguide_estimation_tests estimation_tests.cpp)
target_link_libraries(falconguide_estimation_tests PRIVATE FalconGuide::core FalconGuide::estimation
                                                           FalconGuide::logger)
add_test(NAME falconguide_estimation_tests COMMAND falconguide_estimation_tests)
