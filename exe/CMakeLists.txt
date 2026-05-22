add_executable(falconguide_cli falconguide_cli.cpp)

target_link_libraries(falconguide_cli
  PRIVATE
    FalconGuide::app
    FalconGuide::logger
)

set_target_properties(falconguide_cli PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
