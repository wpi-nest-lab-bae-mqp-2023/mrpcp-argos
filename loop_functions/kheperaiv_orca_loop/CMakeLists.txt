link_directories(${CMAKE_BINARY_DIR}/controllers/kheperaiv_orca)
link_directories(${CMAKE_BINARY_DIR}/loop_functions/collision_handling_loop)
find_package(fmt REQUIRED)

add_library(kheperaiv_orca_loop MODULE kheperaiv_orca_loop.h kheperaiv_orca_loop.cpp id_qtuser_functions.h
        id_qtuser_functions.cpp)
target_link_libraries(kheperaiv_orca_loop
        kheperaiv_orca
        collision_handling_loop
        argos3core_simulator
        argos3plugin_simulator_entities
        argos3plugin_simulator_kheperaiv
        argos3plugin_simulator_genericrobot
        argos3plugin_simulator_qtopengl
        fmt::fmt
)
