cmake_minimum_required(VERSION 3.21)
project(dary_heap VERSION 1.0.0 )
set(CMAKE_CXX_STANDARD 20)

set(SOURCE_FILES src/dary_heap.h)

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

add_executable( test_heap test/test_dary_heap.cpp)
# Link test executable against gtest & gtest_main

target_link_libraries(test_heap gtest gtest_main)
add_test( runUnitTests test_heap)