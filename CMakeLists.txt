//this cmake file replaces that long compilation command
cmake_minimum_required(VERSION 3.21)
project(maze_app LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Tell CMake where to find SFML and other libs
include_directories(
    ${CMAKE_SOURCE_DIR}  # if you include local headers
    "C:/Users/koa/Desktop/SFML-2.6.1-windows-gcc-13.1.0-mingw-64-bit/SFML-2.6.1/include"
    "C:/Users/koa/Desktop/maze_generator/libserialport/include"
)

link_directories(
    "C:/Users/koa/Desktop/SFML-2.6.1-windows-gcc-13.1.0-mingw-64-bit/SFML-2.6.1/lib"
    "C:/Users/koa/Desktop/maze_generator/libserialport/lib"
)

# Add your executable
add_executable(maze_app
    mazeGenerator.cpp
)

# Find SFML
find_package(SFML 2.5 REQUIRED COMPONENTS graphics window system)

# Link against SFML + your other libs
target_link_libraries(maze_app
    sfml-graphics
    sfml-window
    sfml-system
    serialport
    setupapi
    cfgmgr32
    opengl32
    glu32
)
