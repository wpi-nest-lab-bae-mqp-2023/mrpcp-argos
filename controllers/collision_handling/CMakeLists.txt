find_package(fmt REQUIRED)


add_library(collision_handling SHARED collision_mqp.h collision_mqp.cpp PIDController.h PIDController.cpp)
target_link_libraries(collision_handling
        argos3core_simulator
        argos3plugin_simulator_kheperaiv
        argos3plugin_simulator_genericrobot
        fmt::fmt)
