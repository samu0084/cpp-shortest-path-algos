//
// Created by nicolaj on 4/2/22.
//
#include <chrono>
#include <queue>
#include "gtest/gtest.h"
#include "../src/post-order_heap.hpp"

TEST(PerformanceTest, ExpectBetterSequentialPushPerformance) {
    auto comparator = [](int left, int right) {
        return left < right;
    };
    postorder_heap<3, int, std::vector<int>, decltype(comparator)> heap(comparator);
    std::priority_queue<int, std::vector<int>, decltype(comparator)> queue(comparator);

    // Generate numbers
    int* numbers = new int[200000000];
    srand(426086546);
    for (int index = 0; index < 200000000; index++) {
        numbers[index] = rand();
    }

    // Push to heap...
    auto startHeap = std::chrono::high_resolution_clock::now();
    for (int index = 0; index < 2000000; index++)
        heap.push(numbers[index]);
    auto stopHeap = std::chrono::high_resolution_clock::now();
    auto deltaHeap = duration_cast<std::chrono::microseconds>(stopHeap - startHeap);

    // Push to queue
    auto startQueue = std::chrono::high_resolution_clock::now();
    for (int index = 0; index < 2000000; index++)
        queue.push(numbers[index]);
    auto stopQueue = std::chrono::high_resolution_clock::now();
    auto deltaQueue = duration_cast<std::chrono::microseconds>(stopQueue - startQueue);

    EXPECT_TRUE(deltaQueue > deltaHeap);
    if (! (deltaQueue > deltaHeap)) {
        std::cout << "Delta Queue : " << deltaQueue.count() << " and " << " Delta Heap " << deltaHeap.count() << "\n";
    }
}