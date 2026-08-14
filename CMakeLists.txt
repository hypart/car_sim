cmake_minimum_required(VERSION 3.10)
project(car)
set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

include(FetchContent)
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
)
FetchContent_MakeAvailable(raylib)

FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.1
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(eigen)

add_executable(car main.cpp environment.cpp)
target_link_libraries(car PRIVATE raylib Eigen3::Eigen)