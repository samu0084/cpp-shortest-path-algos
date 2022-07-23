//
// Created by asha on 5/12/22.
//

#include "bfs.h"
#include "../graph/graph.h"

#include "bfs.h"

enum Color { WHITE, GRAY, BLACK };

std::pair<std::vector<int>, std::vector<int>> BFS::run(Graph &graph, int source){
    int infinity = std::numeric_limits<int>::max();

    std::vector<Color> color(graph.vertices(), Color::WHITE);
    std::queue<int> queue;
    std::vector<int> distance(graph.vertices(),infinity);
    std::vector<int> parent(graph.vertices(),-1);

    color[source] = Color::GRAY;
    distance[source] = 0;
    queue.push(source);

    while (!queue.empty()) {
        int origin = queue.front();
        queue.pop();
        for(auto edge : graph.getOutgoing(origin)) {
            int target = edge.first;
            if(color[target] == Color::WHITE){
                color[target] = Color::GRAY;
                distance[target] = distance[origin] + 1;
                parent[target] = origin;
                queue.push(target);
            }
            color[origin] = Color::BLACK;
        }
    }
    return {distance, parent};
}

