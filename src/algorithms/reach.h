//
// Created by sx5 on 5/5/2022.
//

#ifndef ATB_REACH_H
#define ATB_REACH_H

#include <set>
#include <map>
#include <list>
#include <chrono>
#include <functional>
#include <utility>
#include "../../modules/post-order-heap/src/post-order_heap.hpp"
#include "../graph/graph.h"
#include "fast_vector.hpp"

struct Results {
    std::vector<std::string> info;
    std::vector<int> bounds;
};

class Reach {
public:
    /**
     * Constructs a reach object from a vector of reaches.
     * @param reaches
     */
    explicit Reach(std::vector<int> &reaches);

    /**
     * Reads reaches from files. Note that the single number on each line, i,
     * is interpreted as the reach of node i, starting from zero. The read
     * reaches is loaded into empty_read_reaches_container, and a reference
     * for the same container is returned to allow function call nesting.
     * @param reach_stream
     * @param empty_read_reaches_container
     * @return
     */
    static std::vector<int> &read(std::istream &reach_stream, std::vector<int> &empty_read_reaches_container);

    /**
     * Write the results to two files at the specified output_path.
     * @param output_path Path to the folder where the files will be made/truncated
     * @param file_postfix Used to create the names: "reach_bounds_[file_postfix].txt" and "info_[file_postfix].txt"
     * @param results Contains the bounds and the info-lines which will be written
     * @param last_line The last line which will be written to the info-file
     */
    static void write(std::string &output_path, std::string &file_postfix, std::vector<int> &bounds);

    /**
     * Tests whether the vertex could potentially be on the shortest-path towards the destination given the path-length
     * to the vertex and the estimates remaining distance_origin to the destination.
     */
    bool test(int vertex, int distance_origin, int distance_destination);

    /**
     * Tests whether the vertex could potentially be on the shortest-path towards the destination given the path-length
     * to the vertex and the estimates remaining distance to the destination.
     */
    bool test(int vertex, int distance_origin, int destination, Graph &graph);

    /**
     * Performs reach-bound computation in iterations and returns the final results
     * @param bound_search_limiters the series of bound-search limiters which will be used
     * @param empty_bounds_for_output must be initialized to the max int value or valid bounds
     */
    static std::vector<int> &iterativeReachBoundComputation(Graph &graph, std::vector<int> &bound_search_limiters, std::vector<int> &initial_bounds);

    /**
     * Performs a single iteration of reach-bound computation
     * @param graph
     * @param bound_search_limiters the series of bound-search limiters which will be used
     * @param empty_bounds_for_output must be initialized to the max int value or valid bounds
     */
    static std::vector<int> &singleReachBoundComputation(Graph &graph, int bound_search_limiter, std::vector<int> &initial_bounds);

    /**
     * Performs a single iteration of the reach bound computation
     * @param graph_of_bounded_plus_outgoing Holds all is_bounded vertices, and the edges between them, and the outgoing
     *                                       edges to unbounded vertices.
     * @param graph_of_unbounded Holds all unbounded vertices and the edges between them.
     * @param graph_of_unbounded_plus_outgoing Holds all unbounded vertices, and the edges between them, as well as the
     *                                         edges which goes from an unbounded vertex to a is_bounded vertex.
     * @param bounds Hold the current bounds on input and is updated to also hold the new on output.
     * @param bound_search_limiter Limits the search-space of each bound-computation before the algorithm stops, assigns
     *                             bounds where possible and removes is_bounded vertices from later computation.
     * @param info Used to output how long various parts of the computation takes
     */
    static void iterate(Graph &graph,
                        std::vector<bool> &is_bounded,
                        std::vector<int> &bounds,
                        int bound_search_limiter);

private:
    std::vector<int> &reaches_;

    /**
     * Computes the maximum value in bounds which is not infinity, or zero if no such bound exists
     */
    static int maxKnownBound(std::vector<int> &bounds);

    /**
     * Computes r(v, T), where T is the tree with distances as provided and where vertex v has the provided descendant leafs.
     */
    static int reachInTree(int vertex, fast_vector<int> &distances, const std::list<int> &descendant_leafs);

    /**
     * Updates the reach-bound of vertex if there is a path with bounds[s] + w(s, v) > bounds[v] and bounds[t] + w(v, t) > bounds[v]
     */
    static void improveReachBound(int vertex,
                                  fast_vector<int> &distances,
                                  std::list<int> &descendant_leafs,
                                  std::vector<bool> &is_bounded,
                                  std::vector<int> &bounds,
                                  int max_ingoing_bound);

    /**
     * A specialized dijkstra for reach-bound computation, which prevents tree growth beyond what is necessary for the given bound
     * search limiter. All structures should have size graph.vertices()
     * @param distances must be initialized to int max
     * @param parents must be initialized to -1
     * @param visited must be initialized to false
     * @param visited_in_order must be initialized to -1
     * @param first_edge_length no initialization needed
     * @param graph Graph for the search
     * @param is_bounded should obey that is_bounded[v] = true if bounds[v] != int max, and false otherwise
     * @param origin Origin of the search
     * @param search_limiter 2 * bound_search_limiter + max_known_bound + max_ingoing_edge_length
     */
    static void dijkstra(fast_vector<int> &distances, fast_vector<int> &parents, fast_vector<bool> &visited, fast_vector<int> &visited_in_order,
                         std::vector<int> &first_edge_length, Graph &graph, const std::vector<bool>& is_bounded, int origin, int search_limiter);

    friend class Reach_dijkstra_Test;
    friend class Reach_dijkstraWithBounds_Test;
};


#endif //ATB_REACH_H
