cmake_minimum_required(VERSION 3.10.0)

project(PBDLearn_example)

### Configure output locations
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# == Dependents
include_directories(external/eigen)
add_subdirectory(external/polyscope)

# == Build our project stuff

set(SRCS 
  cpp/main.cpp
	# add any other source files here
)


# To change the name of your executable, change "gc_project" in the lines below to whatever you want
add_executable(PBDLearn_example "${SRCS}")
target_include_directories(PBDLearn_example PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/cpp/include/")
# add the args.hxx project which we use for command line args
target_include_directories(PBDLearn_example PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/external/polyscope/deps/args")
target_link_libraries(PBDLearn_example polyscope)
