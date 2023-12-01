cmake_minimum_required(VERSION 3.4.1)
project(SDOC-Package)

# settings
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -fno-rtti -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# sub
add_subdirectory(./FuzzyCulling)
add_subdirectory(./src/main/cpp)
