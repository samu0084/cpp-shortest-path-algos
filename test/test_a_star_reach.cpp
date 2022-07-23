//
// Created by sx5 on 5/9/2022.
//

#include <vector>
#include <fstream>
#include <random>
#include "gtest/gtest.h"
#include "../src/graph/graph.h"
#include "../src/algorithms/a_star.h"
#include "../src/algorithms/a_star_reach.h"

TEST(AStarReach, count) {
    std::ifstream v_stream((std::string)  "C:/Users/sx5/CLionProjects/ATB/resources/denmark/nodes.txt", std::ios::in);
    std::ifstream e_stream((std::string)  "C:/Users/sx5/CLionProjects/ATB/resources/denmark/edges.txt", std::ios::in);
    std::cout<<"Load graph."<<std::endl;
    Graph graph = Graph(v_stream, e_stream);
    std::cout << "vertices: " << graph.vertices() << std::endl;
    std::cout << "edges: " << graph.edges() << std::endl;
}

TEST(AStarReach, Specifics) {
    std::vector<int> origins{11001, 17726, 21650, 11050, 10996};
    std::vector<int> destinations{40265, 44021, 25406, 39464, 30744};

    std::ifstream v_stream((std::string) TEST_RESOURCE_DIR + "/nodes.txt", std::ios::in);
    std::ifstream e_stream((std::string) TEST_RESOURCE_DIR + "/edges.txt", std::ios::in);

    Graph graph = Graph(v_stream, e_stream);

    std::vector<int> bound_search_limiters {50, 100, 500};
    std::vector<int> bounds(graph.vertices(), INT_MAX);
    Reach::iterativeReachBoundComputation(graph, bound_search_limiters, bounds);
    Reach reach(bounds);

    for (int iteration = 0; iteration < origins.size(); ++iteration) {
        int origin = origins[iteration];
        int destination = destinations[iteration];

        static auto euclideanDistanceHeuristic = [&graph, &destination](int id) {
            return (int) std::ceil(computeWeight(
                    graph.getCoordinates(id),graph.getCoordinates(destination)));
        };
        static auto zeroHeuristic = [](int id) { return 0; };



        AStarReach algorithm(euclideanDistanceHeuristic, reach);
        AStar control(zeroHeuristic);

        // Run algorithm and extract parents and distances.
        Data data = algorithm.run(graph, origin, destination);
        Data controlData = control.run(graph, origin, destination);

        std::list<int> data_path = AStarReach::extractPath(std::get<1>(data), destination);
        std::list<int> data_path_control = AStar::extractPath(std::get<1>(controlData), destination);
        auto distances = std::get<0>(data);
        auto distances_control = std::get<0>(controlData);

        EXPECT_EQ(distances[destination], distances_control[destination]);

        //std::cout << "Algorithm Path 1 length: "  <<distances[destination] << '\n';
        //std::cout << "  Control Path 2 length: "  << distances_control[destination] << '\n';
    }
}

TEST(AStarReach, DistanceAndReachHeuristicAgreesWithZeroHeuristicSingle) {
    int origin = 1342;
    int destination = 33;

    std::ifstream v_stream((std::string) TEST_RESOURCE_DIR + "/nodes.txt", std::ios::in);
    std::ifstream e_stream((std::string) TEST_RESOURCE_DIR + "/edges.txt", std::ios::in);

    Graph graph = Graph(v_stream, e_stream);

    static auto euclideanDistanceHeuristic = [&graph, &destination](int id) {
        return (int) std::ceil(computeWeight(
                graph.getCoordinates(id),graph.getCoordinates(destination)));
    };
    static auto zeroHeuristic = [](int id) { return 0; };

    std::vector<int> bound_search_limiters {50, 100, 500};
    std::vector<int> bounds(graph.vertices(), INT_MAX);
    std::cout << "Computing bounds..." << std::endl;
    Reach::iterativeReachBoundComputation(graph, bound_search_limiters, bounds);
    Reach reach(bounds);

    AStarReach algorithm(euclideanDistanceHeuristic, reach);
    AStar control(zeroHeuristic);

    // Run algorithm and extract parents and distances.
    Data data = algorithm.run(graph, origin, destination);
    Data controlData = control.run(graph, origin, destination);

    std::list<int> data_path = AStarReach::extractPath(std::get<1>(data), destination);
    std::list<int> data_path_control = AStar::extractPath(std::get<1>(controlData), destination);
    auto distances = std::get<0>(data);
    auto distances_control = std::get<0>(controlData);

    EXPECT_EQ(distances[destination], distances_control[destination]);

    //std::cout << "Algorithm Path 1 length: "  <<distances[destination] << '\n';
    //std::cout << "  Control Path 2 length: "  << distances_control[destination] << '\n';
}

TEST(AStarReach, ReachWithAStarAgreesWithDijkstra) {
    std::ifstream v_stream((std::string) TEST_RESOURCE_DIR + "/nodes.txt", std::ios::in);
    std::ifstream e_stream((std::string) TEST_RESOURCE_DIR + "/edges.txt", std::ios::in);
    std::cout<<"Load graph."<<std::endl;
    Graph graph = Graph(v_stream, e_stream);

    /*std::cout<<"Load reach-bounds."<<std::endl;
    std::ifstream reach_stream((std::string) TEST_RESOURCE_DIR + "/bornholm/reach_bounds_1000.txt", std::ios::in);
    std::vector<int> reach_bounds;
    Reach::read(reach_stream, reach_bounds);
    Reach reach(reach_bounds);*/

    std::vector<int> bound_search_limiters{50,100,250,500,1000};
    std::vector<int> bounds(graph.vertices(), INT_MAX);
    Reach::iterativeReachBoundComputation(graph, bound_search_limiters, bounds);
    Reach reach(bounds);

    std::mt19937 gen(0); // The none-randomness of the seed is intentional. The tests should be reproducible.
    std::uniform_int_distribution<> uniformIntDistributionRange(0, graph.vertices()-1);


    int iterations = 1000;
    for (int iteration = 0; iteration < iterations; ++iteration) {
        int origin = uniformIntDistributionRange(gen);
        int destination = uniformIntDistributionRange(gen);

        static auto euclideanDistanceHeuristic = [&graph, &destination](int id) {
            return (int) std::ceil(computeWeight(
                    graph.getCoordinates(id),graph.getCoordinates(destination)));
        };
        static auto zeroHeuristic = [](int id) { return 0; };

        AStarReach algorithm(euclideanDistanceHeuristic, reach);
        AStar control(zeroHeuristic);

        // Run algorithm and extract parents and distances.
        Data data = algorithm.run(graph, origin, destination);
        Data controlData = control.run(graph, origin, destination);

        std::list<int> data_path = AStarReach::extractPath(std::get<1>(data), destination);
        std::list<int> data_path_control = AStar::extractPath(std::get<1>(controlData), destination);
        auto distances = std::get<0>(data);
        auto distances_control = std::get<0>(controlData);
        EXPECT_EQ(distances[destination], distances_control[destination]);
    }
}

TEST(AStarReach, ReachWithAStarAgreesWithAStar) {
    std::ifstream v_stream((std::string) TEST_RESOURCE_DIR + "/nodes.txt", std::ios::in);
    std::ifstream e_stream((std::string) TEST_RESOURCE_DIR + "/edges.txt", std::ios::in);
    std::cout<<"Load graph."<<std::endl;
    Graph graph = Graph(v_stream, e_stream);

    /*std::cout<<"Load reach-bounds."<<std::endl;
    std::ifstream reach_stream((std::string) TEST_RESOURCE_DIR + "/bornholm/reach_bounds_1000.txt", std::ios::in);
    std::vector<int> reach_bounds;
    Reach::read(reach_stream, reach_bounds);
    Reach reach(reach_bounds);*/

    std::vector<int> bound_search_limiters{50,100,250,500,1000};
    std::vector<int> bounds(graph.vertices(), INT_MAX);
    Reach::iterativeReachBoundComputation(graph, bound_search_limiters, bounds);
    Reach reach(bounds);


    std::mt19937 gen(0); // The none-randomness of the seed is intentional. The tests should be reproducible.
    std::uniform_int_distribution<> uniformIntDistributionRange(0, graph.vertices()-1);


    int iterations = 1000;
    for (int iteration = 0; iteration < iterations; ++iteration) {
        int origin = uniformIntDistributionRange(gen);
        int destination = uniformIntDistributionRange(gen);

        static auto euclideanDistanceHeuristic = [&graph, &destination](int id) {
            return (int) std::ceil(computeWeight(
                    graph.getCoordinates(id),graph.getCoordinates(destination)));
        };

        AStarReach algorithm(euclideanDistanceHeuristic, reach);
        AStar control(euclideanDistanceHeuristic);

        // Run algorithm and extract parents and distances.
        Data data = algorithm.run(graph, origin, destination);
        Data controlData = control.run(graph, origin, destination);

        std::list<int> data_path = AStarReach::extractPath(std::get<1>(data), destination);
        std::list<int> data_path_control = AStar::extractPath(std::get<1>(controlData), destination);
        auto distances = std::get<0>(data);
        auto distances_control = std::get<0>(controlData);
        EXPECT_EQ(distances[destination], distances_control[destination]);
    }
}

TEST(AStarReach, ReachWithoutExternalHeuristicAgreesWithDijkstra) {
    std::ifstream v_stream((std::string) TEST_RESOURCE_DIR + "/nodes.txt", std::ios::in);
    std::ifstream e_stream((std::string) TEST_RESOURCE_DIR + "/edges.txt", std::ios::in);
    std::cout<<"Load graph."<<std::endl;
    Graph graph = Graph(v_stream, e_stream);

    /*std::cout<<"Load reach-bounds."<<std::endl;
    std::ifstream reach_stream((std::string) TEST_RESOURCE_DIR + "/bornholm/reach_bounds_1000.txt", std::ios::in);
    std::vector<int> reach_bounds;
    Reach::read(reach_stream, reach_bounds);
    Reach reach(reach_bounds);*/

    std::vector<int> bound_search_limiters{50,100,250,500,1000};
    std::vector<int> bounds(graph.vertices(), INT_MAX);
    Reach::iterativeReachBoundComputation(graph, bound_search_limiters, bounds);
    Reach reach(bounds);

    std::mt19937 gen(0); // The none-randomness of the seed is intentional. The tests should be reproducible.
    std::uniform_int_distribution<> uniformIntDistributionRange(0, graph.vertices()-1);


    int iterations = 1000;
    for (int iteration = 0; iteration < iterations; ++iteration) {
        int origin = uniformIntDistributionRange(gen);
        int destination = uniformIntDistributionRange(gen);

        static auto euclideanDistanceHeuristic = [&graph, &destination](int id) {
            return (int) computeWeight(
                    graph.getCoordinates(id),graph.getCoordinates(destination));
        };
        static auto zeroHeuristic = [](int id) { return 0; };

        AStarReach algorithm(euclideanDistanceHeuristic, reach);
        AStar control(zeroHeuristic);

        // Run algorithm and extract parents and distances.
        Data data = algorithm.runWithoutExternalHeuristic(graph, origin, destination);
        Data controlData = control.run(graph, origin, destination);

        std::list<int> data_path = AStarReach::extractPath(std::get<1>(data), destination);
        std::list<int> data_path_control = AStar::extractPath(std::get<1>(controlData), destination);
        auto distances = std::get<0>(data);
        auto distances_control = std::get<0>(controlData);
        EXPECT_EQ(distances[destination], distances_control[destination]);
    }
}


