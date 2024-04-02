link_directories(${CMAKE_BINARY_DIR}/controllers/kheperaiv_orca)
find_package(fmt REQUIRED)

add_library(kheperaiv_orca_loop MODULE kheperaiv_orca_loop.h kheperaiv_orca_loop.cpp)
target_link_libraries(kheperaiv_orca_loop
        kheperaiv_orca
        argos3core_simulator
        argos3plugin_simulator_kheperaiv
        argos3plugin_simulator_genericrobot
        fmt::fmt
)
