find_package(CURL REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(fmt REQUIRED)

set(get_initial_solution_mqp_SOURCES
        get_initial_solution_mqp.h
        get_initial_solution_mqp.cpp
        mqp_http_client.h
        mqp_http_client.cpp)

include_directories(
  ${CURL_INCLUDE_DIRS}
)

add_library(get_initial_solution_mqp MODULE
  ${get_initial_solution_mqp_SOURCES})

target_link_libraries(get_initial_solution_mqp
        argos3core_simulator
        argos3plugin_simulator_entities
        argos3plugin_simulator_footbot
        ${CURL_LIBRARIES}
        nlohmann_json::nlohmann_json
        fmt::fmt)
