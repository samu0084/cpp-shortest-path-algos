//
// Created by nicolaj on 3/12/22.
//

#ifndef ATB_GRAPH_H

#include <utility>
#include <tuple>
#include <iostream>
#include <cstring>
#include <cmath>
#include <vector>

#define ATB_GRAPH_H





typedef std::pair<double, double> WGS84;
typedef std::pair<int, int> Edge;

class Graph {
public:
    Graph() = default;
    /**
     * Constructs a graph object from file.
     * @param edges
     */
    Graph(std::istream& v_stream, std::istream& e_stream);

    /**
     * Constructs a graph object from a list of vertex ids and edges with weights. Meant for testing purposes.
     * @param vertices Represented by id
     * @param edges Represented by int source, int target, int weight
     */
    Graph(std::vector<int> &vertices, std::vector<std::tuple<int, int, int>> &edges);

    /**
     *
     * @param vertices Represented by int id, float latitude, float longitude
     * @param edges Represented by int source, int target
     */
    Graph(std::vector<std::tuple<int, float, float>> &vertices, std::vector<std::tuple<int, int>> &edges);

    /**
     * A constructor for making a copy of a other_graph, or an empty other_graph which can hold edges for the same vertices.
     * @param other_graph Another other_graph
     * @param keepEdges Set to True to make a copy that includes the edges, or false to leave all edges out
     */
    Graph(Graph &other_graph, bool keepEdges);

    /**
     * Returns a vector containing the out-going edges of the specified vertex.
     * @param vertex_id int
     * @return vector of edges
     */
    std::vector<Edge>& getOutgoing(int vertex_id);

    /**
     * Returns a vector containing the in-going edges of the specified vertex.
     * @param vertex_id int
     * @return vector of edges
     */
    std::vector<Edge>& getIncoming(int vertex_id);

    /**
     * @brief Get the coordinates of vertex with id vertex_id;
     * 
     * @param vertex_id 
     * @return WGS84 
     */
    WGS84 getCoordinates(int vertex_id);

    /**
     * @brief Create an edge from source to target with weight.
     * 
     * @param origin 
     * @param destination 
     * @param weight 
     */
    void createEdge(int source, int target, int weight);
    /**
     * @brief Removes an edge from source to target
     * 
     * @param origin 
     * @param destination 
     */
    void removeEdge(int source, int target);

    /**
     * Returns the number of vertices in the graph.
     * @return int
     */
    int vertices();

    /**
     * Returns the number of edges in the graph.
     * @return int
     */
    int edges();

    /**W
     * Computes the size of the graph in bytes.
     * @return unsigned long - bytes.
     */
    unsigned long bytes();
private:
    class Vertex {
    public:
        Vertex();
        Vertex(int id, float lat, float lon);
        void addOutgoing(int id, int weight);
        void addIncoming(int id, int weight);
        std::vector<Edge>& getIncoming();
        std::vector<Edge>& getOutgoing();
        WGS84 getCoordinates();
        int getId();
    private:
        int id_;
        float lat_;
        float lon_;
        std::vector<Edge> incoming_;
        std::vector<Edge> outgoing_;
        friend Graph::Graph(Graph &other_graph, bool keepEdges);
    };
    int number_of_edges_;
    int number_of_vertices_;
    std::vector<Vertex> vertices_;
};

// Auxiliary functions :

/**
 * Counts the number of new-lines in a stream.
 * @param stream
 * @return
 */
long count(std::istream &stream);

/**
 * Computes the weight of edges, using a version of Haversine Great Sphere formula adapted for floating point precision.
 * The computed distance is measured in meters.
 * @param origin the integer ID of the origin node.
 * @param destination the integer ID of the destination node.
 * @param nodes a vector containing the lat/lon pairs of all nodes.
 * @return
 */
double computeWeight(WGS84 origin, WGS84 destination);

#endif //ATB_GRAPH_H
