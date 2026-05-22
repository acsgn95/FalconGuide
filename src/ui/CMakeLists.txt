if(NOT FALCONGUIDE_BUILD_UI)
  return()
endif()

# ── Collect Dear ImGui sources (vendored via FetchContent) ────────────────────
FetchContent_GetProperties(imgui SOURCE_DIR IMGUI_SRC_DIR)
FetchContent_GetProperties(implot SOURCE_DIR IMPLOT_SRC_DIR)

add_library(falconguide_ui_imgui STATIC
  ${IMGUI_SRC_DIR}/imgui.cpp
  ${IMGUI_SRC_DIR}/imgui_draw.cpp
  ${IMGUI_SRC_DIR}/imgui_tables.cpp
  ${IMGUI_SRC_DIR}/imgui_widgets.cpp
  ${IMGUI_SRC_DIR}/backends/imgui_impl_glfw.cpp
  ${IMGUI_SRC_DIR}/backends/imgui_impl_opengl3.cpp
  ${IMPLOT_SRC_DIR}/implot.cpp
  ${IMPLOT_SRC_DIR}/implot_items.cpp
)

target_include_directories(falconguide_ui_imgui PUBLIC
  ${IMGUI_SRC_DIR}
  ${IMGUI_SRC_DIR}/backends
  ${IMPLOT_SRC_DIR}
)

target_link_libraries(falconguide_ui_imgui PUBLIC glfw OpenGL::GL ${CMAKE_DL_LIBS})

# ── FalconGuide UI library ────────────────────────────────────────────────────
add_library(falconguide_ui_lib STATIC
  src/ws_client.cpp
  src/ui_app.cpp
)
add_library(FalconGuide::ui ALIAS falconguide_ui_lib)

target_include_directories(falconguide_ui_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

find_package(Threads REQUIRED)

target_link_libraries(falconguide_ui_lib PUBLIC
  falconguide_ui_imgui
  FalconGuide::app
  nlohmann_json::nlohmann_json
  Threads::Threads
)

if(WIN32)
  target_link_libraries(falconguide_ui_lib PRIVATE ws2_32)
endif()
