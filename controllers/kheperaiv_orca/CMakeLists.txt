link_directories(${CMAKE_BINARY_DIR}/controllers/kheperaiv_mqp)

add_library(rvo STATIC IMPORTED)
set_target_properties(rvo PROPERTIES
        IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/.cget/cget/pkg/rvo/install/lib/libRVO.a"
)
target_include_directories(rvo INTERFACE "${CMAKE_SOURCE_DIR}/.cget/cget/pkg/rvo/install/include")

add_library(kheperaiv_orca SHARED kheperaiv_orca.h kheperaiv_orca.cpp Filter.h Filter.cpp)
target_link_libraries(kheperaiv_orca
        kheperaiv_mqp
        rvo
        argos3core_simulator
        argos3plugin_simulator_kheperaiv
        argos3plugin_simulator_genericrobot)
