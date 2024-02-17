find_package(CURL REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

set(mqp1_loop_functions_SOURCES
        mqp1_loop_functions.h
        mqp1_loop_functions.cpp)

include_directories(
  ${CURL_INCLUDE_DIRS}
)

add_library(mqp1_loop_functions MODULE
  ${mqp1_loop_functions_SOURCES})

target_link_libraries(mqp1_loop_functions
        argos3core_simulator
        argos3plugin_simulator_entities
        argos3plugin_simulator_footbot
        ${CURL_LIBRARIES}
        nlohmann_json::nlohmann_json)
