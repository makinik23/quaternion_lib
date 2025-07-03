cmake_minimum_required(VERSION 3.20)
project(quaternionlib LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -pedantic -Werror)

include_directories(include)

add_library(quaternionlib
    src/Quaternion.cpp
)

add_executable(quaternion_tests tests/main.cpp)
target_link_libraries(quaternion_tests PRIVATE quaternionlib)
