# --- CMake Version ---
cmake_minimum_required(VERSION 3.5)

# --- Target ---
add_library(libcola)
add_library(${PROJECT_NAME}::libcola ALIAS libcola)
target_include_directories(
    libcola
    PUBLIC
    ${PROJECT_SOURCE_DIR}/cola/
)
# --- Sources ---
target_sources(
    libcola
    PRIVATE
    box.cpp
    cc_clustercontainmentconstraints.cpp
    cc_nonoverlapconstraints.cpp
    cluster.cpp
    cola.cpp
    compound_constraints.cpp
    conjugate_gradient.cpp
    connected_components.cpp
    convex_hull.cpp
    gradient_projection.cpp
    output_svg.cpp
    pseudorandom.cpp
    shapepair.cpp
    straightener.cpp
)