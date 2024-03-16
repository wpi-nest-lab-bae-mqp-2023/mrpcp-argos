find_package(CURL REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(fmt REQUIRED)


add_library(footbot_mqp MODULE footbot_mqp.h footbot_mqp.cpp mqp_http_client.h mqp_http_client.cpp)
target_link_libraries(footbot_mqp
        argos3core_simulator
        argos3plugin_simulator_footbot
        argos3plugin_simulator_genericrobot
        ${CURL_LIBRARIES}
        nlohmann_json::nlohmann_json
        fmt::fmt)
