//
// Created by nicolaj on 4/3/22.
//

#ifndef ATB_A_STAR_H
#define ATB_A_STAR_H

#include <list>
#include <chrono>
#include <functional>
#include <utility>
#include "../../modules/post-order-heap/src/post-order_heap.hpp"
#include "binary_heap_topdown.hpp"
#include "../graph/graph.h"


typedef std::tuple<std::vector<int>, std::vector<int>, int, std::list<std::pair<int, int>>> Data;

class AStar {
protected:
    typedef int Heuristic(int);
    std::function<Heuristic> heuristic_;
public:
    /**
     * Creates a new AStar object with the specified heuristic.
     * @param heuristic
     */
    explicit AStar(std::function<Heuristic> heuristic);

    /**
     * @brief Runs the AStar algorithm on the referenced graph, starting at origin, and terminating early if destination is found.
     * Note: Setting destination = -1 allows running as single source all destinations.
     * @param graph
     * @param origin
     * @param destination
     * @return (weights, parents, visits)
     */
    Data run(Graph &graph, int origin, int destination);
    /**
     * @brief Runs the AStar algorithm on the referenced graph, starting at origin, and terminating early if destination is found. Uses top-down binary heap.
     * Note: Setting destination = -1 allows running as single source all destinations.
     * @param graph
     * @param origin
     * @param destination
     * @return (weights, parents, visits)
     */    
    Data run_binary(Graph &graph, int origin, int destination);

    /**
     * @brief Runs the AStar algorithm backwards on the referenced graph, starting at origin, and terminating early if destination is found.
     * Note: Setting destination = -1 allows running as single source all destinations.
     * @param graph
     * @param origin
     * @param destination
     * @return (weights, parents, visits)
     */
    Data reverse(Graph &graph, int origin, int destination);

    virtual std::string toString() {
        return "A*";
    }

    /**
     * @brief Extracts the path from destination to origin of the parents array.
     * @param parent
     * @param destination
     * @return
     */
    static std::list<int> extractPath(const std::vector<int>& parent, int destination);
};

#endif //ATB_A_STAR_H