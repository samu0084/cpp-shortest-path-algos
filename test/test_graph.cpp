//
// Created by nicolaj on 4/14/22.
//
#include <fstream>
#include "gtest/gtest.h"
#include "../src/graph/graph.h"

TEST(Graph, CanCreateGraph) {
    std::ifstream v_stream((std::string) TEST_RESOURCE_DIR + "/nodes.txt", std::ios::in);
    std::ifstream e_stream((std::string) TEST_RESOURCE_DIR + "/edges.txt", std::ios::in);

    Graph graph = Graph(v_stream, e_stream);

    EXPECT_EQ(graph.edges(), 104703);
    EXPECT_EQ(graph.vertices(),51051);

    unsigned int edge_count = 0;
    for (int vertex_id = 0; vertex_id < 51051; vertex_id++)
        for (auto in_edge : graph.getIncoming(vertex_id)) {
            edge_count++;
            bool is_also_outgoing = false;
            for (auto out_edge : graph.getOutgoing(in_edge.first)) {
                is_also_outgoing |= out_edge.first == vertex_id; 
            }
            EXPECT_TRUE(is_also_outgoing);
        }
    EXPECT_EQ(edge_count, 104703);
}
TEST(Graph, CanAddEdge) {
    std::ifstream v_stream((std::string) TEST_RESOURCE_DIR + "/nodes.txt", std::ios::in);
    std::ifstream e_stream((std::string) TEST_RESOURCE_DIR + "/edges.txt", std::ios::in);

    Graph graph = Graph(v_stream, e_stream);

    graph.createEdge(0, 100, 1337);
    bool found_incoming = false;
    for (auto edge : graph.getIncoming(100)) {
        if (edge == Edge(0, 1337)) {
            found_incoming = true;
        }
    }
    EXPECT_TRUE(found_incoming);
    bool found_outgoing = false;
    for (auto edge : graph.getOutgoing(0)) {
        if (edge == Edge(100, 1337)) {
            found_outgoing = true;
        }
    }
    EXPECT_TRUE(found_outgoing);
}
TEST(Graph, CanDeleteEdge) {
    std::ifstream v_stream((std::string) TEST_RESOURCE_DIR + "/nodes.txt", std::ios::in);
    std::ifstream e_stream((std::string) TEST_RESOURCE_DIR + "/edges.txt", std::ios::in);

    Graph graph = Graph(v_stream, e_stream);

    graph.createEdge(0, 100, 1337);
    graph.removeEdge(0, 100);
    bool found_incoming = false;
    for (auto edge : graph.getIncoming(100)) {
        if (edge == Edge(0, 1337)) {
            found_incoming = true;
        }
    }
    EXPECT_FALSE(found_incoming);
    bool found_outgoing = false;
    for (auto edge : graph.getOutgoing(0)) {
        if (edge == Edge(100, 1337)) {
            found_outgoing = true;
        }
    }
    EXPECT_FALSE(found_outgoing);
}