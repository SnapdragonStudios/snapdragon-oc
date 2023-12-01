cmake_minimum_required(VERSION 3.4.1)
project(QMeshReducer)

# settings
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -fno-rtti -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# sub
add_subdirectory(./src)



# Add main.cpp file of project root directory as source file
set(SOURCE_FILES src/main.cpp src/MeshReducer.h  src/MeshReducer.cpp)

# Add executable target with source files listed in SOURCE_FILES variable
add_executable(MeshReducer ${SOURCE_FILES})