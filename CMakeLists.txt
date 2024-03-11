# --- CMake Version ---
cmake_minimum_required(VERSION 3.5)

# --- Project ---
project(adaptagrams LANGUAGES CXX)

# --- Build Options ---
option(${PROJECT_NAME}_BUILD_DOCU "Build Documentation" OFF)
option(${PROJECT_NAME}_BUILD_TEST "Build Test" OFF)

# --- Modules ---
add_subdirectory(cola)

# --- Target ---
add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(
    ${PROJECT_NAME}
    INTERFACE
    ${PROJECT_NAME}::libavoid
    ${PROJECT_NAME}::libcola
    ${PROJECT_NAME}::libdialect
    ${PROJECT_NAME}::libproject
    ${PROJECT_NAME}::libtopology
    ${PROJECT_NAME}::libvpsc
)