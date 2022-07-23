#include "gtest/gtest.h"
#include "../src/algorithms/fast_vector.hpp"

TEST(fArray, CanCreateWithCapacity) {
    fast_vector<int> test(10, 42);
    EXPECT_EQ(test[0], 42);
    EXPECT_EQ(test[1], 42);
    EXPECT_EQ(test[2], 42);
    EXPECT_EQ(test[3], 42);
    EXPECT_EQ(test[4], 42);
    EXPECT_EQ(test[5], 42);
    EXPECT_EQ(test[6], 42);
    EXPECT_EQ(test[7], 42);
    EXPECT_EQ(test[8], 42);
    EXPECT_EQ(test[9], 42);
}

TEST(fArray, CanReassign) {
    fast_vector<int> test(10, 42);
    EXPECT_EQ(test[3], 42);
    test[3] = 43;
    EXPECT_EQ(test[3], 43);
}

TEST(fArray, CanReset) {
    fast_vector<int> test(10, 42);
    test[1] = 43;
    test[2] = 43;
    test[3] = 43;
    test[4] = 43;
    EXPECT_EQ(test[3], 43);
    test.reset();
    EXPECT_EQ(test[3], 42);
}
