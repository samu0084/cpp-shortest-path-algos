#include <fstream>
#include "gtest/gtest.h"
#include "../modules/post-order-heap/src/post-order_heap.hpp"
#include "../src/graph/graph.h"
#include "../src/algorithms/a_star.h"
#include "../src/algorithms/landmark.h"
#include "../src/algorithms/bidirectional_a_star.h"

TEST(Landmark, AgreesWithAStar) {
    int origin = 113;
    int destination = 0;

    std::ifstream v_stream((std::string) TEST_RESOURCE_DIR + "/nodes.txt", std::ios::in);
    std::ifstream e_stream((std::string) TEST_RESOURCE_DIR + "/edges.txt", std::ios::in);

    Graph graph = Graph(v_stream, e_stream);

    typedef bool comparator(int, int);
    typedef postorder_heap<2, int, std::vector<int>, std::function<comparator>> QueueType;
    std::vector<int> ids{ 133, 7777, 43153, 9999, 111, 50000 };

    Landmark landmark(graph);
    landmark.generate(ids);
    landmark.activate(ids);
    landmark.setDestination(destination);

    static auto landmarkHeuristic = [&landmark](int id) {
        return landmark.apply(id);
    };
    AStar algorithm(landmarkHeuristic);
    static auto zeroHeuristic = [](int id) {
        return 0;
    };
    AStar control(zeroHeuristic);

    // Run algorithm and extract parents and distances.
    Data data = algorithm.run(graph, origin, destination);
    Data controlData = control.run(graph, origin, destination);

    std::list<int> path1 = AStar::extractPath(std::get<1>(data), destination);
    std::list<int> path2 = AStar::extractPath(std::get<1>(controlData),  destination);

    EXPECT_EQ(path1.size(), path2.size());
    EXPECT_EQ(std::get<0>(data)[destination], std::get<0>(controlData)[destination]);
    EXPECT_LE(std::get<2>(data), std::get<2>(controlData));
}

TEST(Landmark, WorksWithBidirectional) {
    std::ifstream v_stream((std::string) TEST_RESOURCE_DIR + "/nodes.txt", std::ios::in);
    std::ifstream e_stream((std::string) TEST_RESOURCE_DIR + "/edges.txt", std::ios::in);

    Graph graph = Graph(v_stream, e_stream);

    int origin = 113;
    int destination = 941;

    std::vector<int> ids{ 133, 931, 921 };

    Landmark landmark(graph);
    landmark.generate(ids);
    landmark.activate(ids);

    static auto f_heuristic = [&landmark, destination](int id) {
        landmark.setDestination(destination);
        return landmark.apply(id);
    };
    static auto r_heuristic = [&landmark, origin](int id) {
        landmark.setDestination(origin);
        return landmark.apply(id);
    };

    // Prepare bidirectional.
    typedef bool comparator(int, int);
    typedef postorder_heap<2, int, std::vector<int>, std::function<comparator>> QueueType;
    static auto zeroHeuristic = [](int id) {
        return 0;
    };
    BidirectionalAStar bidirectional(f_heuristic, r_heuristic);
    AStar unidirectional(zeroHeuristic);
    // Run algorithm and extract parents and distances.
    BidirectionalData data = bidirectional.run(graph, origin, destination);
    Data controlData = unidirectional.run(graph, origin, destination);

    std::list<int> path1 = bidirectional.extractPath(std::get<2>(data),std::get<3>(data), std::get<4>(data));
    std::list<int> path2 = AStar::extractPath(std::get<1>(controlData),  destination);
    EXPECT_EQ(path1.size(), path2.size());
    EXPECT_EQ((std::get<1>(data)[std::get<4>(data)] + std::get<0>(data)[std::get<4>(data)]), std::get<0>(controlData)[destination]);
    EXPECT_LE(std::get<5>(data), std::get<2>(controlData));
}