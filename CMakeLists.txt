cmake_minimum_required(VERSION 3.22)
project(maze_app LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find SFML (Standard Linux location)
find_package(SFML 2.5 REQUIRED COMPONENTS graphics window system)

# Find libserialport (Standard Linux location)
# Unlike SFML, libserialport usually needs a manual search on Linux
find_path(SERIALPORT_INCLUDE_DIR libserialport.h)
find_library(SERIALPORT_LIBRARY serialport)

add_executable(maze_app src/mazeGenerator.cpp)

# Link everything
target_include_directories(maze_app PRIVATE ${SERIALPORT_INCLUDE_DIR})
target_link_libraries(maze_app 
    sfml-graphics 
    sfml-window 
    sfml-system 
    ${SERIALPORT_LIBRARY}
)