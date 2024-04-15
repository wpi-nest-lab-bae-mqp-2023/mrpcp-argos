link_directories(${CMAKE_BINARY_DIR}/controllers/collision_handling)
include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
find_package(CURL REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

add_library(collision_handling_loop SHARED
        mqp_http_client.h
        collision_handling_loop.h
        data_parsing.h
        mqp_http_client.cpp
        collision_handling_loop.cpp
        data_parsing.cpp)

target_link_libraries(collision_handling_loop
        collision_handling
        argos3core_simulator
        argos3plugin_simulator_entities
        argos3plugin_simulator_footbot
        ${CURL_LIBRARIES}
        nlohmann_json::nlohmann_json
        argos3plugin_simulator_qtopengl
        ${ARGOS_QTOPENGL_LIBRARIES})
