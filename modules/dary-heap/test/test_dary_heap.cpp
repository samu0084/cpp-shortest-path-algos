#include "gtest/gtest.h"
#include "../src/dary_heap.h"

TEST(Heap, CanInsert) {
    auto comparator = [](int left, int right) {
        return left < right;
    };
    dary_heap<2, int, std::vector<int>, decltype(comparator)> queue(comparator);
    queue.push(10);
    EXPECT_EQ(queue.top(), 10);
    queue.pop();
}

TEST(Heap, CanSort) {
    auto comparator = [](int left, int right) {
        return left < right;
    };
    dary_heap<4, int, std::vector<int>, decltype(comparator)> queue(comparator);
    std::vector<int> numbers = {3, 77, 1, 23, 0, 333, -1, 99, 32, 11, 9999, 123, 321, 34566, 66754, 3112346,987,455443};
    for (auto number : numbers) {
        queue.push(number);
    }
    EXPECT_EQ(queue.top(), -1);
    int top = queue.top();
    queue.pop();
    while (!queue.empty()) {
        int pop = queue.top();
        queue.pop();
        EXPECT_GE(pop, top);
        top = pop;
    }
}