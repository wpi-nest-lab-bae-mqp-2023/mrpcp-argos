find_package(fmt REQUIRED)


add_library(footbot_mqp SHARED footbot_mqp.h footbot_mqp.cpp)
target_link_libraries(footbot_mqp
        argos3core_simulator
        argos3plugin_simulator_footbot
        argos3plugin_simulator_genericrobot
        fmt::fmt)
