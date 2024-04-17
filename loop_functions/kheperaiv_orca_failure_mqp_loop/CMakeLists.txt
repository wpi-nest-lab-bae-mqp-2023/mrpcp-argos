link_directories(${CMAKE_BINARY_DIR}/controllers/kheperaiv_orca_failure_mqp)
find_package(CURL REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

add_library(kheperaiv_orca_failure_mqp_loop MODULE kheperaiv_orca_failure_mqp_loop.h kheperaiv_orca_failure_mqp_loop.cpp qtuser_functions.h qtuser_functions.cpp mqp_http_client.h mqp_http_client.cpp position_logger.h position_logger.cpp)
target_link_libraries(kheperaiv_orca_failure_mqp_loop
        kheperaiv_orca_failure_mqp
        argos3core_simulator
        argos3plugin_simulator_entities
        argos3plugin_simulator_kheperaiv
        argos3plugin_simulator_genericrobot
        argos3plugin_simulator_qtopengl
        ${CURL_LIBRARIES}
        nlohmann_json::nlohmann_json
)
