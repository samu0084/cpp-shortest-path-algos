//
// Created by nicolaj on 4/12/22.
//
#include "gtest/gtest.h"
#include "../modules/cpp-arrays/src/array.hpp"
#include "../modules/memory-map-controller/src/memory_map_controller.h"

TEST(MemoryMappedArrays, CanCreateWithCapacity) {
    memory_map_controller controller;
    int* mapped = controller.falloc<int>(10000000, MADV_WILLNEED | MADV_HUGEPAGE | MADV_SEQUENTIAL);
    arrays::array<int> array = arrays::array<int>(mapped, 10000000);
    array.fill(42);
    for (auto integer : array) {
        EXPECT_EQ(42, integer);
    }
}

TEST(MemoryMappedArrays, CanReassign) {
    memory_map_controller controller;
    int* mapped = controller.falloc<int>(10, MADV_WILLNEED);
    arrays::array<int> array = arrays::array<int>(mapped, 10);
    array.fill(42);
    array[5] = 4321;
    EXPECT_EQ(array[5], 4321);
}