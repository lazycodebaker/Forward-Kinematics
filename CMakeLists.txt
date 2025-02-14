cmake_minimum_required(VERSION 3.10)

set(CMAKE_C_COMPILER /usr/bin/clang)
set(CMAKE_CXX_COMPILER /usr/bin/clang++)

project(inverse_kinematics)

# Find OpenCV
set(OpenCV_DIR "/opt/homebrew/Cellar/opencv/4.11.0_1/")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# Find SFML 3 with explicit components
set(SFML_DIR "/opt/homebrew/Cellar/sfml/3.0.0/lib/cmake/SFML")
find_package(SFML 3 COMPONENTS Graphics Window System REQUIRED)
include_directories(/opt/homebrew/Cellar/sfml/3.0.0/include)
link_directories(/opt/homebrew/Cellar/sfml/3.0.0/lib)

# Add Eigen include directory
include_directories(/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3)

# Compiler Flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
string(REPLACE "--no-as-needed" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
string(REPLACE "--as-needed" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

# Define executable
add_executable(inverse_kinematics inverse_kinematics.cpp)

# Link against OpenCV, and SFML
target_link_libraries(inverse_kinematics  
    ${OpenCV_LIBS} 
    SFML::Graphics 
    SFML::Window 
    SFML::System
)

# Set C++ Standard
set_property(TARGET inverse_kinematics PROPERTY CXX_STANDARD 17)