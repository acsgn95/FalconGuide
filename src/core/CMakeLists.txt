add_library(falconguide_core INTERFACE)
add_library(FalconGuide::core ALIAS falconguide_core)

target_include_directories(
  falconguide_core
  INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
  falconguide_core
  INTERFACE
    Eigen3::Eigen
)
