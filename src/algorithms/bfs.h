//
// Created by asha on 5/12/22.
//

#ifndef ATB_BFS_H
#define ATB_BFS_H

#include <queue>
#include "../graph/graph.h"

class BFS {
public:
    /**
     * A standard bfs implementation.
     * @param graph
     * @param source
     * @return rank vector and parent vector.
     */
    static std::pair<std::vector<int>, std::vector<int>> run(Graph &graph, int source);
};


#endif //ATB_BFS_H