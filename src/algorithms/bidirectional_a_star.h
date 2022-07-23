//
// Created by nicolaj on 4/18/22.
//

#ifndef ATB_BIDIRECTIONAL_A_STAR_H
#define ATB_BIDIRECTIONAL_A_STAR_H

#include <functional>
#include <chrono>
#include <list>
#include "../../modules/post-order-heap/src/post-order_heap.hpp"
#include "../graph/graph.h"

typedef std::tuple<std::vector<int>, std::vector<int>, std::vector<int>, std::vector<int>, int, int, std::list<std::pair<int, int>>, std::list<std::pair<int, int>>> BidirectionalData;


class BidirectionalAStar {
private:
    typedef int Heuristic(int);
    std::function<Heuristic> forward_heuristic_;
    std::function<Heuristic> backward_heuristic_;
public:
    explicit BidirectionalAStar(std::function<Heuristic> forwardHeuristic, std::function<Heuristic> backwardHeuristic);

    BidirectionalData run(Graph& graph, int origin, int destination);

    virtual std::string toString() {
        return "Bidirectional A*";
    }

    std::list<int> extractPath(std::vector<int> forward_parent, std::vector<int> backward_parent, int meeting_point);
};

#endif //ATB_BIDIRECTIONAL_A_STAR_H
