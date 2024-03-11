cmake_minimum_required(VERSION 3.5)

add_library(libavoid)
add_library(${PROJECT_NAME}::libavoid ALIAS libavoid)
target_include_directories(
    libavoid
    PUBLIC
    ${PROJECT_SOURCE_DIR}/cola/
)
target_sources(
    libavoid
    PRIVATE
    actioninfo.cpp
    connectionpin.cpp
    connector.cpp
    connend.cpp
    geometry.cpp
    geomtypes.cpp
    graph.cpp
    hyperedge.cpp
    hyperedgeimprover.cpp
    hyperedgetree.cpp
    junction.cpp
    makepath.cpp
    mtst.cpp
    obstacle.cpp
    orthogonal.cpp
    router.cpp
    scanline.cpp
    shape.cpp
    timer.cpp
    vertices.cpp
    viscluster.cpp
    visibility.cpp
    vpsc.cpp
)