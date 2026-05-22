add_library(falconguide_app STATIC src/config_json.cpp src/dataset_reader.cpp src/ipc_server.cpp src/ws_server.cpp
                                   src/session.cpp)
add_library(FalconGuide::app ALIAS falconguide_app)

target_include_directories(falconguide_app PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(falconguide_app PUBLIC FalconGuide::core FalconGuide::estimation FalconGuide::io
                                             FalconGuide::logger nlohmann_json::nlohmann_json)
