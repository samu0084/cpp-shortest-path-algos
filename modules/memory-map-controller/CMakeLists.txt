cmake_minimum_required(VERSION 3.21)
project(memory_map_controller VERSION 1.0.0 )
set(CMAKE_CXX_STANDARD 20)

set(SOURCE_FILES src/memory_map_controller.cpp)

include(FetchContent)

add_library(${PROJECT_NAME} ${SOURCE_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE src)

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION})

########################
##   Google Testing  ###
########################
FetchContent_Declare(
        googletest
        # Specify the commit you depend on and update it regularly.
        URL https://github.com/google/googletest/archive/e2239ee6043f73722e7aa812a459f54a28552929.zip
)
FetchContent_MakeAvailable(googletest)
add_test( runUnitTests testArray)