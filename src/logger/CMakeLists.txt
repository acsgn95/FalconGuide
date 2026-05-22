add_library(falconguide_logger STATIC
  src/logger.cpp
)
add_library(FalconGuide::logger ALIAS falconguide_logger)

target_include_directories(falconguide_logger
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(falconguide_logger
  PUBLIC
    spdlog::spdlog
)

# Expose SPDLOG_ACTIVE_LEVEL = TRACE so callers can control verbosity at compile time.
target_compile_definitions(falconguide_logger
  PUBLIC
    SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE
)
