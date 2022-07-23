//
// Created by nicolaj on 4/19/22.
//
#include "a_star.h"

AStar::AStar(std::function<Heuristic> heuristic) : heuristic_(std::move(heuristic)) {}


Data AStar::run(Graph &graph, int origin, int destination) {
    int visits = 0;
    int number_vertices = graph.vertices();

    // Initialise aux. datastructures
    int infinity = std::numeric_limits<int>::max();
    std::vector<int> distance(number_vertices, infinity);
    std::vector<int> key(number_vertices, infinity);
    std::vector<bool> visited(number_vertices, false);
    std::vector<int> parent(number_vertices, -1);
    std::list<std::pair<int, int>> edge_order;

    // Initialise queue compare using 'key' -> distance from origin + heuristic.
    auto comparator = [&key](int left, int right) {
        return (key[left]) < (key[right]);
    };
    postorder_heap<3, int, std::vector<int>, decltype(comparator)> queue(comparator);

    distance[origin] = 0;
    key[origin] = heuristic_(origin);
    queue.push(origin);
    while (!queue.empty()) {
        int local_source = queue.top();
        queue.pop();

        if (local_source == destination)
            break;

        if (!visited[local_source]) {
            visited[local_source] = true;
            visits++;

            for (auto& edge : graph.getOutgoing(local_source)) {
                int local_target = edge.first;
                int weight = edge.second;
                int tentative_distance = distance[local_source] + weight;
                edge_order.emplace_back(local_source, local_target);

                // If current edge allows for a shorter route -> relax
                if (tentative_distance < distance[local_target]) {
                    distance[local_target] = tentative_distance;
                    key[local_target] = tentative_distance + heuristic_(local_target);
                    parent[local_target] = local_source;
                    queue.push(local_target);
                }
            }
        }
    }
    return {distance, parent, visits, edge_order};
}

Data AStar::reverse(Graph &graph, int origin, int destination) {
    int visits = 0;
    int number_vertices = graph.vertices();

    // Initialise aux. datastructures
    int infinity = std::numeric_limits<int>::max();
    std::vector<int> distance(number_vertices, infinity);
    std::vector<int> key(number_vertices, infinity);
    std::vector<bool> visited(number_vertices, false);
    std::vector<int> parent(number_vertices, -1);
    std::list<std::pair<int, int>> edge_order;

    // Initialise queue compare using 'key' -> distance from origin + heuristic.
    auto comparator = [&key](int left, int right) {
        return (key[left]) < (key[right]);
    };
    postorder_heap<3, int, std::vector<int>, decltype(comparator)> queue(comparator);

    distance[origin] = 0;
    key[origin] = heuristic_(origin);
    queue.push(origin);
    while (!queue.empty()) {
        int local_source = queue.top();
        queue.pop();

        if (local_source == destination)
            break;

        if (!visited[local_source]) {
            visited[local_source] = true;
            visits++;

            for (auto& edge : graph.getIncoming(local_source)) {
                int local_target = edge.first;
                int weight = edge.second;
                int tentative_distance = distance[local_source] + weight;
                edge_order.emplace_back(local_source, local_target);

                // If current edge allows for a shorter route -> relax
                if (tentative_distance < distance[local_target]) {
                    distance[local_target] = tentative_distance;
                    key[local_target] = tentative_distance + heuristic_(local_target);
                    parent[local_target] = local_source;
                    queue.push(local_target);
                }
            }
        }
    }
    return {distance, parent, visits, edge_order};
}

std::list<int> AStar::extractPath(const std::vector<int>& parent, int destination) {
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

Data AStar::run_binary(Graph &graph, int origin, int destination) {
    int visits = 0;
    int number_vertices = graph.vertices();

    // Initialise aux. datastructures
    int infinity = std::numeric_limits<int>::max();
    std::vector<int> distance(number_vertices, infinity);
    std::vector<int> key(number_vertices, infinity);
    std::vector<bool> visited(number_vertices, false);
    std::vector<int> parent(number_vertices, -1);
    std::list<std::pair<int, int>> edge_order;

    // Initialise queue compare using 'key' -> distance from origin + heuristic.
    auto comparator = [&key](int left, int right) {
        return (key[left]) < (key[right]);
    };
    binary_heap_topdown<int, std::vector<int>, decltype(comparator)> queue(comparator);

    distance[origin] = 0;
    key[origin] = heuristic_(origin);
    queue.push(origin);
    while (!queue.empty()) {
        int local_source = queue.top();
        queue.pop();

        if (local_source == destination)
            break;

        if (!visited[local_source]) {
            visited[local_source] = true;
            visits++;

            for (auto& edge : graph.getOutgoing(local_source)) {
                int local_target = edge.first;
                int weight = edge.second;
                int tentative_distance = distance[local_source] + weight;
                edge_order.emplace_back(local_source, local_target);

                // If current edge allows for a shorter route -> relax
                if (tentative_distance < distance[local_target]) {
                    distance[local_target] = tentative_distance;
                    key[local_target] = tentative_distance + heuristic_(local_target);
                    parent[local_target] = local_source;
                    queue.push(local_target);
                }
            }
        }
    }
    return {distance, parent, visits, edge_order};
}