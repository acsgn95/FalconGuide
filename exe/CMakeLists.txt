add_executable(falconguide_cli falconguide_cli.cpp)

target_link_libraries(falconguide_cli PRIVATE FalconGuide::app FalconGuide::logger)

set_target_properties(falconguide_cli PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

if(FALCONGUIDE_BUILD_UI)
  add_executable(falconguide_ui falconguide_ui.cpp)
  target_link_libraries(falconguide_ui PRIVATE FalconGuide::ui)
  set_target_properties(falconguide_ui PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endif()
