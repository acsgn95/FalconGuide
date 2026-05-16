add_library(falconguide_estimation INTERFACE)
add_library(FalconGuide::estimation ALIAS falconguide_estimation)

target_include_directories(
  falconguide_estimation
  INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
  falconguide_estimation
  INTERFACE
    FalconGuide::core
)
