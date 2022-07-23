//
// Created by sx5 on 5/9/2022.
//

#ifndef ATB_A_STAR_REACH_H
#define ATB_A_STAR_REACH_H

#include <list>
#include <chrono>
#include <functional>
#include <utility>
#include "../../modules/post-order-heap/src/post-order_heap.hpp"
#include "../graph/graph.h"
#include "reach.h"

typedef std::tuple<std::vector<int>, std::vector<int>, int, std::list<std::pair<int, int>>> Data;

class AStarReach {
private:
    typedef int Heuristic(int);
    std::function<Heuristic> heuristic_;
    Reach reach_;
public:
    /**
     * Creates a new AStarReach object with the specified heuristic.
     * This heuristic will only be used when calling the 'run' method.
     * @param heuristic
     * @param reach
     */
    explicit AStarReach(std::function<Heuristic> heuristic, Reach reach);

    /**
     * @brief Runs the AStar algorithm on the referenced graph, starting
     * at origin, and terminating if destination is found or cannot be reached.
     * @param graph
     * @param origin
     * @param destination
     * @return (weights, parents, visits)
     */
    Data run(Graph &graph, int origin, int destination);

    /**
     * @brief Runs the Reach algorithm without the AStar. The algorithm starts
     * at origin, and terminates if destination is found or cannot be reached.
     * @param graph
     * @param origin
     * @param destination
     * @return (weights, parents, visits)
     */
    Data runWithoutExternalHeuristic(Graph &graph, int origin, int destination);

    virtual std::string toString() {
        return "Reach";
    }

    /**
     * @brief Extracts the path from destination to origin of the parents array.
     * @param parent
     * @param destination
     * @return
     */
    static std::list<int> extractPath(const std::vector<int>& parent, int destination);
};

#endif //ATB_A_STAR_REACH_H
