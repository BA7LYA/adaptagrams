# --- CMake Version ---
cmake_minimum_required(VERSION 3.5)

# --- Target ---
add_library(libvpsc)
add_library(${PROJECT_NAME}::libvpsc ALIAS libvpsc)
target_include_directories(
    libvpsc
    PUBLIC
    ${PROJECT_SOURCE_DIR}/cola/
)
# --- Sources ---
target_sources(
    libvpsc
    PRIVATE
    block.cpp
    blocks.cpp
    cbuffer.cpp
    constraint.cpp
    rectangle.cpp
    solve_VPSC.cpp
    variable.cpp
)