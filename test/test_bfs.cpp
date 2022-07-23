//
// Created by asha on 5/13/22.
//

#include "../src/algorithms/bfs.h"
#include <fstream>
#include "gtest/gtest.h"




TEST(BFS, SimpleTest) {
    // Construct simple graph
    int source = 0;

    std::ifstream v_stream((std::string) TEST_RESOURCE_DIR + "/manuel_nodes.txt", std::ios::in);
    std::ifstream e_stream((std::string) TEST_RESOURCE_DIR + "/manuel_edges.txt", std::ios::in);

    Graph graph = Graph(v_stream, e_stream);

    // Run bfs
    std::pair<std::vector<int>, std::vector<int>> result = BFS::run(graph, source);


    //Test result

    auto dist = result.first;

    EXPECT_EQ(dist[0], 0);
    EXPECT_EQ(dist[1], 1);
    EXPECT_EQ(dist[2], 1);
    EXPECT_EQ(dist[3], 1);
    EXPECT_EQ(dist[4], 2);
    EXPECT_EQ(dist[5], 2);
    EXPECT_EQ(dist[6], 2);
    EXPECT_EQ(dist[7], 2);
    EXPECT_EQ(dist[8], 3);
    EXPECT_EQ(dist[9], 3);
    EXPECT_EQ(dist[10], 3);
    EXPECT_EQ(dist[11], 3);
    EXPECT_EQ(dist[5], 2);
    EXPECT_EQ(dist[3], 1);
}

