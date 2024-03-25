find_package(fmt REQUIRED)


add_library(kheperaiv_mqp SHARED kheperaiv_mqp.h kheperaiv_mqp.cpp)
target_link_libraries(kheperaiv_mqp
        argos3core_simulator
        argos3plugin_simulator_kheperaiv
        argos3plugin_simulator_genericrobot
        fmt::fmt)
