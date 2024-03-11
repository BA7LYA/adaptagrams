# --- CMake Version ---
cmake_minimum_required(VERSION 3.5)

# --- Target ---
add_library(libdialect)
add_library(${PROJECT_NAME}::libdialect ALIAS libdialect)
target_include_directories(
    libdialect
    PUBLIC
    ${PROJECT_SOURCE_DIR}/cola/
)
# --- Sources ---
target_sources(
    libdialect
    PRIVATE
    aca.cpp
    bendseqlookup.cpp
    chains.cpp
    constraints.cpp
    edges.cpp
    expansion.cpp
    faces.cpp
    graphs.cpp
    hola.cpp
    io.cpp
    logging.cpp
    nearalign.cpp
    nexes.cpp
    nodeconfig.cpp
    nodes.cpp
    ortho.cpp
    peeling.cpp
    planarise.cpp
    qalookup.cpp
    quadaction.cpp
    routing.cpp
    sides.cpp
    treeplacement.cpp
    trees.cpp
)