#include <fstream>
#include "gtest/gtest.h"
#include "../src/graph/graph.h"
#include "../src/algorithms/contraction_hierarchies.h"
#include "../src/algorithms/a_star.h"
#include "../src/algorithms/bidirectional_a_star.h"

std::ostream& operator<<(std::ostream& ostr, const std::list<int>& list)
{
    for (auto &i : list) {
        ostr << " " << i;
    }
    return ostr;
}

TEST(CH, CanContract) {
    std::ifstream v_stream((std::string) TEST_RESOURCE_DIR + "/nodes.txt", std::ios::in);
    std::ifstream e_stream((std::string) TEST_RESOURCE_DIR + "/edges.txt", std::ios::in);

    Graph graph = Graph(v_stream, e_stream);
    ContractionHierarchies algorithm;
    algorithm.createContractionHierarchy(graph);
}

TEST(CH, AgreesWithDijkstra) {
    const int iterations = 1000;
    std::ifstream v_stream((std::string) TEST_RESOURCE_DIR + "/nodes.txt", std::ios::in);
    std::ifstream e_stream((std::string) TEST_RESOURCE_DIR + "/edges.txt", std::ios::in);
    Graph graph = Graph(v_stream, e_stream);

    ContractionHierarchies contraction_hierarchy;
    contraction_hierarchy.createContractionHierarchy(graph);
    static auto zeroHeuristic = [](int id) {
        return 0;
    };
    AStar dijkstra(zeroHeuristic);

    for (int iteration = 0; iteration < iterations; iteration++) {
        int destination = 0 + (std::rand() % (graph.vertices() - 1));
        int origin = 0 + (std::rand() % (graph.vertices() - 1));
        Data control_data = dijkstra.run(graph, origin, destination);
        CHData data = contraction_hierarchy.query(origin, destination);
        if (std::get<0>(control_data)[destination] != std::numeric_limits<int>::max()) {
            EXPECT_EQ((std::get<1>(data)[std::get<4>(data)] + std::get<0>(data)[std::get<4>(data)]), std::get<0>(control_data)[destination]);
        }
    }
}

TEST(CH, CanDecompress) {
    std::ifstream v_stream((std::string) TEST_RESOURCE_DIR + "/nodes.txt", std::ios::in);
    std::ifstream e_stream((std::string) TEST_RESOURCE_DIR + "/edges.txt", std::ios::in);
    Graph graph = Graph(v_stream, e_stream);

    ContractionHierarchies contraction_hierarchy;
    contraction_hierarchy.createContractionHierarchy(graph);
    static auto zeroHeuristic = [](int id) {
        return 0;
    };
    BidirectionalAStar dijkstra(zeroHeuristic, zeroHeuristic);

    int destination = 44;
    int origin = 11213;

    auto control_data = dijkstra.run(graph, origin, destination);
    auto data = contraction_hierarchy.query(origin, destination);

    auto control_path = dijkstra.extractPath(std::get<2>(control_data), std::get<3>(control_data), std::get<4>(control_data));
    auto path = contraction_hierarchy.extractPath(std::get<2>(data), std::get<3>(data), std::get<4>(data));
    EXPECT_EQ(control_path.front(), origin);
    EXPECT_EQ(control_path.back(), destination);
    EXPECT_EQ(control_path.front(), path.front());
    EXPECT_EQ(control_path.back(), path.back());
    EXPECT_EQ(control_path.size(), path.size());
}