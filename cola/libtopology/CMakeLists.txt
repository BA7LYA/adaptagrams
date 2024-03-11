# --- CMake Version ---
cmake_minimum_required(VERSION 3.5)

# --- Target ---
add_library(libtopology)
add_library(${PROJECT_NAME}::libtopology ALIAS libtopology)
target_include_directories(
    libtopology
    PUBLIC
    ${PROJECT_SOURCE_DIR}/cola/
)
# --- Sources ---
target_sources(
    libtopology
    PRIVATE
    cola_topology_addon.cpp
    compute_forces.cpp
    orthogonal_topology.cpp
    resize.cpp
    topology_constraints.cpp
    topology_constraints_constructor.cpp
    topology_graph.cpp
)