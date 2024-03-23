link_directories(${CMAKE_BINARY_DIR}/controllers/footbot_mqp2)
include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
find_package(CURL REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(fmt REQUIRED)

add_library(get_initial_solution_mqp2 MODULE
        mqp_http_client.h
        get_initial_solution_mqp2.h
        mqp_http_client.cpp
        get_initial_solution_mqp2.cpp)

target_link_libraries(get_initial_solution_mqp2
        footbot_mqp2
        argos3core_simulator
        argos3plugin_simulator_entities
        argos3plugin_simulator_footbot
        ${CURL_LIBRARIES}
        nlohmann_json::nlohmann_json
        fmt::fmt
        argos3plugin_simulator_qtopengl
        ${ARGOS_QTOPENGL_LIBRARIES})
