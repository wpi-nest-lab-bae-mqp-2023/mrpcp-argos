add_library(kheperaiv_orca MODULE kheperaiv_orca.h kheperaiv_orca.cpp)
target_link_libraries(kheperaiv_orca
        argos3core_simulator
        argos3plugin_simulator_kheperaiv
        argos3plugin_simulator_genericrobot)
