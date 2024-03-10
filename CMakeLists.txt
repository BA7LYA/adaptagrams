# --- CMake Version ---
cmake_minimum_required(VERSION 3.21)

# --- Project ---
project(adaptagrams LANGUAGES CXX)

# --- Build Options ---
option(${PROJECT_NAME}_BUILD_DOC "Build Documentation" OFF)
option(${PROJECT_NAME}_BUILD_TEST "Build Test" OFF)

# --- Includes ---
include(CTest)
include(GoogleTest)

# --- Dependencies ---
find_package(asio CONFIG REQUIRED)
find_package(benchmark CONFIG REQUIRED)
find_package(Boost REQUIRED COMPONENTS graph graph_parallel)
find_package(GTest CONFIG REQUIRED)
find_package(igraph CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(nodesoup CONFIG REQUIRED)
find_package(ogdf CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(Taskflow CONFIG REQUIRED)

find_path(CXXGRAPH_INCLUDE_DIR cxxgraph/CXXGraph.hpp)

# --- OS ---
if (WIN32)
    add_definitions(-DWINVER=0x0A00)       # Windows 10
    add_definitions(-D_WIN32_WINNT=0x0A00) # Windows 10
    add_definitions(-DNOMINMAX)            # disable C2589 min()/max() error
endif()

# --- Dependencies ---
add_subdirectory(thirdparty)

# --- Target ---
add_library(${PROJECT_NAME} INTERFACE)
add_library(ba7lya::graph ALIAS ${PROJECT_NAME})

target_include_directories(
    ${PROJECT_NAME}
    INTERFACE
    ${CXXGRAPH_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/include/
)

target_link_libraries(
    ${PROJECT_NAME}
    INTERFACE
    Adaptagrams
    asio::asio
    Boost::graph
    Boost::graph_parallel
    COIN
    igraph::bliss
    igraph::igraph
    igraph::pcg
    igraph::prpack
    nlohmann_json::nlohmann_json
    nodesoup::nodesoup
    OGDF
    spdlog::spdlog
    Taskflow::Taskflow
)

target_compile_features(
    ${PROJECT_NAME}
    INTERFACE
    cxx_std_23
)

target_compile_definitions(
    ${PROJECT_NAME}
    INTERFACE
    EXPORT_DLL
)

target_compile_options(
    ${PROJECT_NAME}
    INTERFACE
    "$<$<C_COMPILER_ID:MSVC>:/utf-8>"
    "$<$<CXX_COMPILER_ID:MSVC>:/utf-8>"
)

# --- Unit Tests ---
if(${PROJECT_NAME}_BUILD_TEST)
    enable_testing()
    add_subdirectory(test)
endif()

# --- Benchmark ---
if(${PROJECT_NAME}_BUILD_BENCHMARK)
    add_subdirectory(benchmark)
endif()

# --- Demo ---
if(${PROJECT_NAME}_BUILD_DEMO)
    add_subdirectory(demo)
endif()