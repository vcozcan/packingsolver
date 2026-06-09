cmake_minimum_required(VERSION 3.28)

project(PackingSolver LANGUAGES CXX)

# Solver options.
option(PACKINGSOLVER_USE_CLP "Use CLP" ON)
option(PACKINGSOLVER_USE_HIGHS "Use HiGHS" ON)
option(PACKINGSOLVER_USE_KNITRO "Use Knitro" OFF)

# Build options.
option(PACKINGSOLVER_BUILD_MAIN "Build main" ON)
option(PACKINGSOLVER_BUILD_TEST "Build unit tests" ON)
# The box and boxstacks solvers are excluded from the default build: they are
# rarely used here and slow the build down (and currently hit a Windows
# MAX_PATH issue in the deep _deps tree). Re-enable with
# -DPACKINGSOLVER_BUILD_BOX_SOLVERS=ON when a project needs them.
option(PACKINGSOLVER_BUILD_BOX_SOLVERS "Build box and boxstacks solvers" OFF)

# Avoid FetchContent warning.
cmake_policy(SET CMP0135 NEW)

# Require C++14.
set(CMAKE_CXX_STANDARD 14)

# Enable output of compile commands during generation.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set MSVC_RUNTIME_LIBRARY.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

# Add sub-directories.
add_subdirectory(extern)
add_subdirectory(src)
if(PACKINGSOLVER_BUILD_TEST)
    add_subdirectory(test)
endif()
