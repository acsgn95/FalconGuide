find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs features2d calib3d)

add_library(falconguide_vo STATIC src/vo_processor.cpp)
add_library(FalconGuide::vo ALIAS falconguide_vo)

target_include_directories(falconguide_vo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(falconguide_vo PUBLIC opencv_core opencv_imgproc opencv_imgcodecs opencv_features2d
                                            opencv_calib3d)
