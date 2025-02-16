cmake_minimum_required(VERSION 3.10.0)

project(PBDLearn_example)
file(GLOB_RECURSE pbd_cpp_files "${CMAKE_CURRENT_SOURCE_DIR}/cpp/src/*.cpp")

### Configure output locations
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# == Dependents
include_directories(external/eigen)
add_subdirectory(external/polyscope)

add_library(pbd SHARED ${pbd_cpp_files})
target_include_directories(pbd PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include )

# == Build our project stuff

# test_polyscope
add_executable(test_polyscope ${CMAKE_CURRENT_SOURCE_DIR}/cpp/test_polyscope.cpp)
target_include_directories(test_polyscope PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/cpp/include/")
target_link_libraries(test_polyscope polyscope)

# test_wind
add_executable(test_wind ${CMAKE_CURRENT_SOURCE_DIR}/cpp/test_wind.cpp ${pbd_cpp_files})
target_include_directories(test_wind PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/cpp/include/")
target_link_libraries(test_wind pbd polyscope)