add_library(footbot_mqp2 SHARED footbot_mqp2.h footbot_mqp2.cpp)
target_link_libraries(footbot_mqp2
        argos3core_simulator
        argos3plugin_simulator_footbot
        argos3plugin_simulator_genericrobot)
