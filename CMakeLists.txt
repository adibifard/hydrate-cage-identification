cmake_minimum_required(VERSION 3.26)
project(crystal_cage_identification)


set(CMAKE_CXX_STANDARD 20)

# Add header file include directories
include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/
        # Add any additional directories here
)

# Add block directories
#file(GLOB_RECURSE SOURCES
#        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
#)
set(SOURCES
        main.cpp
        InOut.cpp
        HBondedMol.cpp
        AlgCage.cpp
        AlgCage_CageViaCup.cpp
        Auxilaryfun.cpp
        GeneralCage.cpp
        Planes.cpp
        PolygonAreas.cpp
        # Add all other .cpp files you want to compile and link
)

# Define the executable
add_executable(${PROJECT_NAME} ${SOURCES})
