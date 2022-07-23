//
// Created by nicolaj on 4/3/22.
//


#include <list>
#include <chrono>
#include <functional>
#include <utility>
#include "../../modules/post-order-heap/src/post-order_heap.hpp"
#include "../graph/graph.h"


typedef std::tuple<std::vector<int>, std::vector<int>, int, std::list<std::pair<int, int>>> Data;

template<class Queue>
class AStarQ {
private:
    typedef int Heuristic(int);
    std::function<Heuristic> heuristic_;
public:
    /**
     * Creates a new AStar object with the specified heuristic.
     * @param heuristic
     */
    explicit AStarQ(std::function<Heuristic> heuristic) : heuristic_(std::move(heuristic)) {}

    /**
     * @brief Runs the AStar algorithm on the referenced graph, starting at origin, and terminating early if destination is found.
     * Note: Setting destination = -1 allows running as single source all destinations.
     * @param graph
     * @param origin
     * @param destination
     * @return (weights, parents, visits)
     */
    Data run(Graph &graph, int origin, int destination) {
    int visits = 0;
    int number_vertices = graph.vertices();

    // Initialise aux. datastructures
    int infinity = std::numeric_limits<int>::max();
    std::vector<int> distance(number_vertices, infinity);
    std::vector<bool> visited(number_vertices, false);
    std::vector<int> parent(number_vertices, -1);
    std::list<std::pair<int, int>> edge_order;

    Queue queue;

    distance[origin] = 0;
    queue.push({0, origin});
    while (!queue.empty()) {
        int local_origin = queue.top().second;
        queue.pop();

        if (local_origin == destination)
            break;

        if (!visited[local_origin]) {
            visited[local_origin] = true;
            visits++;

            for (auto& edge : graph.getOutgoing(local_origin)) {
                int local_destination = edge.first;
                int weight = edge.second;
                int tentative_distance = distance[local_origin] + weight;
                edge_order.emplace_back(local_origin, local_destination);
                // If current edge allows for a shorter route -> relax
                if (tentative_distance < distance[local_destination]) {
                    distance[local_destination] = tentative_distance;
                    auto key = tentative_distance + heuristic_(local_destination);
                    parent[local_destination] = local_origin;
                    queue.push({tentative_distance, local_destination});
                }
            }
        }
    }
    return {distance, parent, visits, edge_order};
}


    virtual std::string toString() {
        return "A* Queue Safe";
    }

    /**
     * @brief Extracts the path from destination to origin of the parents array.
     * @param parent
     * @param destination
     * @return
     */
    static std::list<int> extractPath(const std::vector<int>& parent, int destination) {
            std::list<int> path;
            // If no path exists...
            if (parent[destination] == -1)
                return path;
            int current = destination;
            while (current != -1) {
                path.emplace_back(current);
                current = parent[current];
            }
            path.reverse();
            return path;
        }
};
