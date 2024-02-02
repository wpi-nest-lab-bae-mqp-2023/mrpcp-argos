pkg_check_modules(CURL libcurl REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

set(mrpcp_loop_functions_SOURCES
  mrpcp_loop_functions.h
  mrpcp_loop_functions.cpp)

include_directories(
  ${CURL_INCLUDE_DIRS}
)

add_library(mrpcp_loop_functions MODULE
  ${mrpcp_loop_functions_SOURCES})

target_link_libraries(mrpcp_loop_functions
        argos3core_simulator
        argos3plugin_simulator_entities
        argos3plugin_simulator_footbot
        ${CURL_LIBRARIES}
        nlohmann_json::nlohmann_json)
