//
// Created by nicolaj on 4/12/22.
//

#include <gtest/gtest.h>
#include "../src/array.hpp"

arrays::array<arrays::array<int>> create() {
  arrays::array<arrays::array<int>> array = arrays::array<arrays::array<int>>(42);
      return array;
}

std::pair<arrays::array<arrays::array<int>>, arrays::array<arrays::array<int>>> createPair() {
  return {create(), create()};
}

TEST(Initialization, ArraysCanBeInitializedWithValue) {
  arrays::array<int> array = arrays::array<int>(40, 42);
  for (int i = 0; i < array.size(); i++) {
    EXPECT_EQ(42, array[i]);
  }
}

TEST(Initialization, ArraysCanBeInitializedFromPointers) {
  int* pointer = new int[40];
  arrays::array<int> array = arrays::array<int>(pointer, 40);
  array.fill(42);
  for (int i = 0; i < array.size(); i++) {
    EXPECT_EQ(42, array[i]);
  }
}

TEST(Assignment, ArrayElementsCanBeReassigned) {
  arrays::array<int> array = arrays::array<int>(40, 42);
  arrays::array<int> copy = arrays::array<int>(array);
  array[10] = 10;
  for (int i = 0; i < array.size(); i++) {
    if (i != 10) {
      EXPECT_EQ(42, array[i]);
    } else {
      EXPECT_EQ(10, array[i]);
    }
  }
}

TEST(Size, ArraysSizeReturnsCapacity) {
  arrays::array<int> array = arrays::array<int>(40);

  EXPECT_EQ(40, array.size());
}

TEST(Move, CanMoveAssign) {
  arrays::array<arrays::array<int>> array = create();
  array[3] = arrays::array<int>(40, 42);
  EXPECT_EQ(array[3][2], 42);
}

TEST(Move, CanMoveConstruct) {
  arrays::array<int> array = arrays::array<int>(40, 42);
  arrays::array<int> moved;
  moved = std::move(array);
}