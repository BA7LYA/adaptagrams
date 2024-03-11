# --- CMake Version ---
cmake_minimum_required(VERSION 3.5)

# --- Target ---
add_library(libproject)
add_library(${PROJECT_NAME}::libproject ALIAS libproject)
target_include_directories(
    libproject
    PUBLIC
    ${PROJECT_SOURCE_DIR}/cola/
)
# --- Sources ---
target_sources(
    libproject
    PRIVATE
    project.cpp
    util.cpp
)