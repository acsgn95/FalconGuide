add_library(falconguide_io STATIC src/nmea/nmea_writer.cpp src/csv/csv_measurement_reader.cpp
                                  src/sensor_logger/sensor_logger_reader.cpp)
add_library(FalconGuide::io ALIAS falconguide_io)

target_include_directories(falconguide_io PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(falconguide_io PUBLIC FalconGuide::core FalconGuide::estimation)
