cmake_minimum_required(VERSION 3.15)
project(cpp_fast LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -funroll-loops")

# Использовать pybind11 из pip (свежий), а не из apt
execute_process(
    COMMAND python3 -c "import pybind11; print(pybind11.get_cmake_dir())"
    OUTPUT_VARIABLE PYBIND11_CMAKE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
list(APPEND CMAKE_PREFIX_PATH "${PYBIND11_CMAKE_DIR}")

find_package(pybind11 REQUIRED)
find_package(Python3 COMPONENTS Development NumPy REQUIRED)

message(STATUS "pybind11 dir: ${pybind11_DIR}")

pybind11_add_module(cpp_fast cpp_fast.cpp)
target_compile_options(cpp_fast PRIVATE -O3 -march=native)