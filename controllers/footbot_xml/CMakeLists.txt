add_library(footbot_xml MODULE footbot_xml.h footbot_xml.cpp)
target_link_libraries(footbot_xml
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
