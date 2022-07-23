//
// Created by nicolaj on 4/18/22.
//
#include <fstream>
#include "gtest/gtest.h"
#include "../src/graph/graph.h"
#include "../src/algorithms/a_star.h"
#include "../modules/post-order-heap/src/post-order_heap.hpp"
#include "../src/algorithms/bidirectional_a_star.h"

TEST(TestBidirectionalAstar, AgreesWithRegular) {
    const int iterations = 1;
    const unsigned int seed = 3456789;

    srand(seed);

    std::ifstream v_stream((std::string) TEST_RESOURCE_DIR + "/nodes.txt", std::ios::in);
    std::ifstream e_stream((std::string) TEST_RESOURCE_DIR + "/edges.txt", std::ios::in);

    Graph graph = Graph(v_stream, e_stream);

    // Prepare bidirectional.
    static auto zeroHeuristic = [](int id) {
        return 0;
    };
    BidirectionalAStar bidirectional(zeroHeuristic, zeroHeuristic);
    AStar unidirectional(zeroHeuristic);

    for (int iteration = 0; iteration < iterations; iteration++) {
        int destination = 0 + (std::rand() % (graph.vertices() - 0 + 1));
        int origin = 0 + (std::rand() % (graph.vertices() - 0 + 1));

        Data data1 = unidirectional.run(graph, origin, destination);
        BidirectionalData data = bidirectional.run(graph, origin, destination);

        std::list<int> path1 = unidirectional.extractPath(std::get<1>(data1), destination);
        std::list<int> path = bidirectional.extractPath(std::get<2>(data), std::get<3>(data), std::get<4>(data));

        EXPECT_EQ(path1.size(), path.size());

        EXPECT_EQ(std::get<0>(data)[std::get<4>(data)] + std::get<1>(data)[std::get<4>(data)], std::get<0>(data1)[destination]);
    }
}

TEST(TestBidirectionalAstar, AutoCheckVisits) {
    const int iterations = 1;
    std::ifstream v_stream((std::string) TEST_RESOURCE_DIR + "/nodes.txt", std::ios::in);
    std::ifstream e_stream((std::string) TEST_RESOURCE_DIR + "/edges.txt", std::ios::in);

    Graph graph = Graph(v_stream, e_stream);

    int destination = 0;
    int origin = 0;


    // Prepare bidirectional.
    static auto f_distance = [&graph, &destination](int id) {
        return (int) std::ceil(computeWeight(graph.getCoordinates(id), graph.getCoordinates(destination)));
    };
    static auto b_distance = [&graph, &origin](int id) {
        return (int) std::ceil(computeWeight(graph.getCoordinates(id), graph.getCoordinates(origin)));
    };
    static auto zeroHeuristic = [](int id) {
        return 0;
    };
    BidirectionalAStar bidirectional(f_distance, b_distance);
    AStar unidirectional(zeroHeuristic);



    for (int iteration = 0; iteration < iterations; iteration++) {
        destination = 0 + (std::rand() % (graph.vertices() - 0 + 1));
        origin = 0 + (std::rand() % (graph.vertices() - 0 + 1));
        Data controlData = unidirectional.run(graph, origin, destination);
        BidirectionalData data = bidirectional.run(graph, origin, destination);
        std::list<int> path2 = unidirectional.extractPath(std::get<1>(controlData), destination);
        std::list<int> path1 = bidirectional.extractPath(std::get<2>(data), std::get<3>(data), std::get<4>(data));
        if (std::get<0>(controlData)[destination] != std::numeric_limits<int>::max()) {
            EXPECT_EQ((std::get<1>(data)[std::get<4>(data)] + std::get<0>(data)[std::get<4>(data)]), std::get<0>(controlData)[destination]);
        }
        else
            EXPECT_EQ((std::get<1>(data)[std::get<4>(data)] + std::get<0>(data)[std::get<4>(data)]), 0);
    }
}