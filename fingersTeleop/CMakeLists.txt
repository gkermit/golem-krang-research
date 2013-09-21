cmake_minimum_required(VERSION 2.8)
project(forwardKinematics)

# Set the C99 standard for the C files
set(CMAKE_INSTALL_PREFIX /usr)
set(CMAKE_CXX_FLAGS "-g -std=gnu++0x")

# Guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds are not allowed. You may need to remove CMakeCache.txt.")
endif()

# Link to somatic, amino and ach
# NOTE: Ideally we would like to 'find' these packages but for now, we assume they are either 
# in /usr/lib or /usr/local/lib
link_libraries(protobuf-c amino ach kore somatic filter stdc++ ncurses)

# Include Eigen
#include_directories(/usr/local/include/eigen3)
#include_directories(/usr/include/eigen3)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)

# Include the project files and the common helpers for all the experiments
include_directories(src)
include_directories(../common)

# Collect the source, script and fortran files
#file(GLOB main_source "src/*.cpp" "src/*.c")
file(GLOB scripts_source "exe/*.cpp")
LIST(SORT scripts_source)

# Build the library and set the executables that use it (have to hardcode for now)
#add_library(Main SHARED ${main_source})

# uncomment to break apart libraries
#add_library(KrangControl SHARED "src/KrangControl.cpp")
#add_library(WorkspaceControl SHARED "src/WorkspaceControl.cpp")

# Build scripts
message(STATUS "\n-- SCRIPTS: ")
foreach(script_src_file ${scripts_source})
	get_filename_component(script_base ${script_src_file} NAME_WE)
	message(STATUS "Adding script ${script_src_file} with base name ${script_base}" )
	add_executable(${script_base} ${script_src_file})
#	target_link_libraries(${script_base} Main) 
	target_link_libraries(${script_base} ${GRIP_LIBRARIES} ${DART_LIBRARIES} ${DARTExt_LIBRARIES} ${wxWidgets_LIBRARIES}) 
	add_custom_target(${script_base}.run ${script_base} ${ARGN})
endforeach(script_src_file)
message(STATUS " ")
