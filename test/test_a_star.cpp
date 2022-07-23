//
// Created by nicolaj on 4/14/22.
//
#include <fstream>
#include <random>
#include "gtest/gtest.h"
#include "../src/graph/graph.h"
#include "../src/algorithms/a_star.h"
#include "../modules/post-order-heap/src/post-order_heap.hpp"

double getWeight(int parent, int node, Graph& graph) {
    for (auto edge : graph.getOutgoing(parent)) {
        int weight = edge.second;
        int destination = edge.first;
        if (destination == node)
            return weight;
    }
    return 0;
}

TEST(AStarReach, AStarAgreesWithDijkstra) {
    std::ifstream v_stream((std::string) TEST_RESOURCE_DIR + "/nodes.txt", std::ios::in);
    std::ifstream e_stream((std::string) TEST_RESOURCE_DIR + "/edges.txt", std::ios::in);

    Graph graph = Graph(v_stream, e_stream);

    std::mt19937 gen(0); // The none-randomness of the seed is intentional. The tests should be reproducible.
    std::uniform_int_distribution<> uniformIntDistributionRange(0, graph.vertices()-1);

    int iterations = 1000;
    for (int iteration = 0; iteration < iterations; ++iteration) {
        int origin = uniformIntDistributionRange(gen);
        int destination = uniformIntDistributionRange(gen);
        //std::cout<<"Test "<<iteration<<". Find "<<origin<<"->"<<destination<<std::endl;

        static auto euclideanDistanceHeuristic = [&graph, &destination](int id) {
            return (int) std::ceil(computeWeight(
                    graph.getCoordinates(id),graph.getCoordinates(destination)));
        };
        static auto zeroHeuristic = [](int id) { return 0; };

        AStar algorithm(euclideanDistanceHeuristic);
        AStar control(zeroHeuristic);

        // Run algorithm and extract parents and distances.
        Data data = algorithm.run(graph, origin, destination);
        Data controlData = control.run(graph, origin, destination);

        std::list<int> data_path = AStar::extractPath(std::get<1>(data), destination);
        std::list<int> data_path_control = AStar::extractPath(std::get<1>(controlData), destination);
        auto distances = std::get<0>(data);
        auto distances_control = std::get<0>(controlData);
        EXPECT_EQ(distances[destination], distances_control[destination]);
    }
}

TEST(AStar, DistanceHeuristicAgreesWithZeroHeuristic) {
    int origin = 1342;
    int destination = 33;

    std::ifstream v_stream((std::string) TEST_RESOURCE_DIR + "/nodes.txt", std::ios::in);
    std::ifstream e_stream((std::string) TEST_RESOURCE_DIR + "/edges.txt", std::ios::in);

    Graph graph = Graph(v_stream, e_stream);

    static auto distance = [&graph, &destination](int id) {
        return (int) std::ceil(computeWeight(graph.getCoordinates(id), graph.getCoordinates(destination)));
    };
    static auto zero = [](int id) {
        return 0;
    };

    AStar algorithm(distance);
    AStar control(zero);

    // Run algorithm and extract parents and distances.
    Data data = algorithm.run(graph, origin, destination);
    Data controlData = control.run(graph, origin, destination);

    std::list<int> path1 = algorithm.extractPath(std::get<1>(data), destination);
    std::list<int> path2 = control.extractPath(std::get<1>(controlData), destination);
    auto distances = std::get<0>(data);
    auto distances_control = std::get<0>(controlData);
    EXPECT_EQ(path1.size(), path2.size());
    EXPECT_EQ(distances[destination],distances_control[destination]);
    EXPECT_LE(std::get<2>(data), std::get<2>(controlData));
}

TEST(AStar, AllDestinationsAStarFormsShortestPathTree) {
    std::ifstream v_stream((std::string) TEST_RESOURCE_DIR + "/nodes.txt", std::ios::in);
    std::ifstream e_stream((std::string) TEST_RESOURCE_DIR + "/edges.txt", std::ios::in);

    Graph graph = Graph(v_stream, e_stream);

    // Prepare algorithm.
    AStar algorithm([](int id) { return 0; });

    // Run n iterations...
    for (int iteration = 0; iteration < 10; iteration++) {
        int origin = 0 + (std::rand() % (graph.vertices() - 0 + 1));

        // Run algorithm and extract parents and distances.
        Data data = algorithm.run(graph, origin, -1);
        auto distances = std::get<0>(data);
        auto parents = std::get<1>(data);

        // Source not in parent tree
        EXPECT_EQ(-1, parents[origin]);
        // Distance to source = 0.
        EXPECT_EQ(0, distances[origin]);

        // For all nodes ...
        for (int local_origin = 0; local_origin < graph.vertices(); local_origin++) {
            // Asert all distances are greater than 0
            EXPECT_TRUE(distances[local_origin] >= 0);
            if (local_origin != origin) {
                int parent = parents[local_origin];
                if (parents[local_origin] == -1) {
                    // Assert that distance to nodes with no parent is infinity
                    EXPECT_EQ(std::numeric_limits<int>::max(), distances[local_origin]);
                } else {
                    // Assert that no branch in the tree has inf weight.
                    EXPECT_EQ((distances[local_origin] - (distances[parent] + getWeight(parent, local_origin, graph))), 0);
                    EXPECT_TRUE(distances[local_origin] < std::numeric_limits<int>::max());
                    // Assert that distances and parents form a shortest path tree
                    for (auto edge : graph.getOutgoing(local_origin)) {
                        int weight = edge.second;
                        int destination = edge.first;
                        EXPECT_LE(distances[destination], distances[local_origin] + weight);
                    }
                }
            }
        }
    }
}


