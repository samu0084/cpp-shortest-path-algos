cmake_minimum_required(VERSION 3.21)
set(CMAKE_CXX_STANDARD 20)
project(ATB VERSION 1.0.0)

include(GNUInstallDirs)

######################
##  Git Submodules  ##
######################
include_directories(${CMAKE_INSTALL_INCLUDEDIR})

function(add_git_submodule dir)
    # add a Git submodule directory to CMake, assuming the
    # Git submodule directory is a CMake project.
    #
    # Usage: in CMakeLists.txt
    #
    # include(AddGitSubmodule.cmake)
    # add_git_submodule(mysubmod_dir)

    find_package(Git REQUIRED)

    if(NOT EXISTS ${dir}/CMakeLists.txt)
        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive -- ${dir}
                WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                COMMAND_ERROR_IS_FATAL ANY)
    endif()

    add_subdirectory(${dir})

endfunction(add_git_submodule)

add_git_submodule(modules/cpp-arrays)
add_git_submodule(modules/post-order-heap)
add_git_submodule(modules/memory-map-controller)
add_git_submodule(modules/dary-heap)

######################
## Internal Modules ##
######################

add_library(graph src/graph/graph.cpp)
target_link_libraries(graph Arrays)
target_compile_definitions(graph PUBLIC TEST_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources")

add_library(algorithms src/algorithms/a_star.h src/algorithms/landmark.cpp src/algorithms/bidirectional_a_star.cpp src/algorithms/a_star.cpp src/algorithms/contraction_hierarchies.cpp src/algorithms/contraction_hierarchies.h src/algorithms/a_star_reach.cpp src/algorithms/reach.cpp src/algorithms/bfs.cpp src/algorithms/bfs.cpp src/algorithms/a_star_queue_safe.hpp src/algorithms/fast_vector.hpp)
target_link_libraries(algorithms graph postorder_heap dary_heap)

add_library(measurements src/measurements/snitch.cpp)

add_library(queue_tests src/test_fixtures/queue_experiments.cpp src/test_fixtures/landmarks_experiments.cpp)
target_link_libraries(queue_tests graph algorithms)


######################
##    Executables   ##
######################

add_executable( run_compare_time_only_test src/test_fixtures/compare_all_time_only.cpp)
target_link_libraries(run_compare_time_only_test gtest gtest_main postorder_heap dary_heap algorithms)
target_compile_definitions(run_compare_time_only_test PUBLIC TEST_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources")

add_executable( run_compare_tests src/test_fixtures/compare_all.cpp)
target_link_libraries(run_compare_tests gtest gtest_main postorder_heap dary_heap measurements algorithms)
target_compile_definitions(run_compare_tests PUBLIC TEST_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources")

add_executable( run_queue_tests src/test_fixtures/queue_experiments.cpp src/algorithms/binary_heap_topdown.hpp)
target_link_libraries(run_queue_tests gtest gtest_main postorder_heap dary_heap measurements algorithms)
target_compile_definitions(run_queue_tests PUBLIC TEST_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources")

add_executable( run_reach_preprocessing_experiments src/test_fixtures/reach_preprocessing_experiments.cpp)
target_link_libraries(run_reach_preprocessing_experiments gtest gtest_main postorder_heap dary_heap algorithms)
target_compile_definitions(run_reach_preprocessing_experiments PUBLIC TEST_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources")

add_executable( run_reach_query_time_experiments src/test_fixtures/reach_query_time_experiments.cpp)
target_link_libraries(run_reach_query_time_experiments gtest gtest_main postorder_heap dary_heap algorithms)
target_compile_definitions(run_reach_query_time_experiments PUBLIC TEST_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources")

add_executable( landmarks_experiments src/test_fixtures/landmarks_experiments.cpp)
target_link_libraries(landmarks_experiments gtest gtest_main measurements algorithms)
target_compile_definitions(landmarks_experiments PUBLIC TEST_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources")

add_executable( generate_plot src/test_fixtures/generate_plots.cpp)
target_link_libraries(generate_plot gtest gtest_main postorder_heap dary_heap algorithms)
target_compile_definitions(generate_plot PUBLIC TEST_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources")

add_executable( run_ch_experiments src/test_fixtures/ch_experiments.cpp)
target_link_libraries(run_ch_experiments gtest gtest_main postorder_heap dary_heap measurements algorithms)
target_compile_definitions(run_ch_experiments PUBLIC TEST_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources")

######################
##  Google Testing  ##
######################
FetchContent_Declare(
        googletest
        # Specify the commit you depend on and update it regularly.
        URL https://github.com/google/googletest/archive/e2239ee6043f73722e7aa812a459f54a28552929.zip
)
FetchContent_MakeAvailable(googletest)

add_executable( test_memory_mapped_arrays test/test_memory_mapped_arrays.cpp)
target_link_libraries(test_memory_mapped_arrays gtest gtest_main Arrays memory_map_controller)
target_compile_definitions(test_memory_mapped_arrays PUBLIC TEST_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources")

add_executable( test_graph test/test_graph.cpp)
target_link_libraries(test_graph gtest gtest_main graph)
target_compile_definitions(test_graph PUBLIC TEST_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources")

add_executable( test_a_star test/test_a_star.cpp)
target_link_libraries(test_a_star gtest gtest_main graph algorithms)
target_compile_definitions(test_a_star PUBLIC TEST_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources")

add_executable( test_a_star_queue test/test_a_star_queue.cpp)
target_link_libraries(test_a_star_queue gtest gtest_main graph algorithms)
target_compile_definitions(test_a_star_queue PUBLIC TEST_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources")


add_executable( test_bidirectional test/test_bidirectional_a_star.cpp)
target_link_libraries(test_bidirectional gtest gtest_main graph algorithms)
target_compile_definitions(test_bidirectional PUBLIC TEST_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources")

add_executable( test_landmarks test/test_landmarks.cpp)
target_link_libraries(test_landmarks gtest gtest_main graph algorithms)
target_compile_definitions(test_landmarks PUBLIC TEST_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources")

add_executable( test_contraction_hierarchies test/test_contraction_hierarchies.cpp)
target_link_libraries(test_contraction_hierarchies gtest gtest_main graph algorithms)
target_compile_definitions(test_contraction_hierarchies PUBLIC TEST_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources")

add_executable( test_astar_reaches test/test_a_star_reach.cpp)
target_link_libraries(test_astar_reaches gtest gtest_main graph algorithms)
target_compile_definitions(test_astar_reaches PUBLIC TEST_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources")

add_executable( test_BFS test/test_bfs.cpp)
target_link_libraries(test_BFS gtest gtest_main graph algorithms)
target_compile_definitions(test_BFS PUBLIC TEST_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources")

add_executable( test_measurements test/test_measurements.cpp)
target_link_libraries(test_measurements gtest gtest_main measurements)
target_compile_definitions(test_measurements PUBLIC TEST_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources")

add_executable( test_fast_vector test/test_fast_array.cpp)
target_link_libraries(test_fast_vector gtest gtest_main algorithms)
target_compile_definitions(test_fast_vector PUBLIC TEST_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources")

add_executable( test_binary_heap test/test_binary_heap_topdown.cpp)
target_link_libraries(test_binary_heap gtest gtest_main algorithms)
target_compile_definitions(test_binary_heap PUBLIC TEST_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources")

add_executable( test_all test/test_contraction_hierarchies.cpp test/test_landmarks.cpp test/test_bidirectional_a_star.cpp test/test_a_star.cpp test/test_graph.cpp test/test_memory_mapped_arrays.cpp test/test_measurements.cpp test/test_bfs.cpp test/test_a_star_queue.cpp test/test_binary_heap_topdown.cpp test/test_fast_array.cpp)
target_link_libraries(test_all gtest gtest_main graph algorithms Arrays memory_map_controller measurements)
target_compile_definitions(test_all PUBLIC TEST_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources")

add_executable( test_reach test/test_reach.cpp)
target_link_libraries(test_reach gtest gtest_main graph algorithms)
target_compile_definitions(test_reach PUBLIC TEST_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources")