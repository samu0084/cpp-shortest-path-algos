cmake_minimum_required(VERSION 3.21)
project(postorder_heap VERSION 1.0)
include(FetchContent)
set(CMAKE_CXX_STANDARD 20)

########################
##   Google Testing  ###
########################
FetchContent_Declare(
        googletest
        # Specify the commit you depend on and update it regularly.
        URL https://github.com/google/googletest/archive/e2239ee6043f73722e7aa812a459f54a28552929.zip
)
FetchContent_MakeAvailable(googletest)

add_library(postorder_heap STATIC src/post-order_heap.hpp)

target_include_directories(${PROJECT_NAME} PRIVATE include)
target_include_directories(${PROJECT_NAME} PRIVATE src)

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION})


########################
##   Unit Testing    ###
########################

add_executable( test_postorder_heap test/test_post_order_heap.cpp test/test_performance.cpp)
# Link test executable against gtest & gtest_main

target_link_libraries(test_postorder_heap gtest gtest_main)
add_test( runUnitTests test_postorder_heap)
