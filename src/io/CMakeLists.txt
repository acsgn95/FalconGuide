add_library(falconguide_io INTERFACE)
add_library(FalconGuide::io ALIAS falconguide_io)

target_include_directories(
  falconguide_io
  INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
  falconguide_io
  INTERFACE
    FalconGuide::core
    FalconGuide::estimation
)
