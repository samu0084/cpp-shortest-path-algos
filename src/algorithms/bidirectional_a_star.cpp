#include "bidirectional_a_star.h"

BidirectionalAStar::BidirectionalAStar(std::function<Heuristic> forwardHeuristic, std::function<Heuristic> backwardHeuristic)
        : forward_heuristic_(std::move(forwardHeuristic)),
          backward_heuristic_(std::move(backwardHeuristic)) { }


BidirectionalData BidirectionalAStar::run(Graph &graph, int source, int target) {
    int visits = 0;

    // Determine number of nodes.
    int number_vertices = graph.vertices();

    // Initialise observed distances with infinity.
    int infinity = std::numeric_limits<int>::max();
    // Initialise aux. datastructures
    std::vector<int> forward_cumulative_weight(number_vertices, infinity);
    std::vector<int> forward_key(number_vertices, infinity);
    std::vector<bool> forward_visited(number_vertices, false);
    std::vector<int> forward_parent(number_vertices, -1);
    std::vector<int> backward_cumulative_weight(number_vertices, infinity);
    std::vector<int> backward_key(number_vertices, infinity);
    std::vector<bool> backward_visited(number_vertices, false);
    std::vector<int> backward_parent(number_vertices, -1);
    std::list<std::pair<int, int>> forward_edge_order;
    std::list<std::pair<int, int>> backward_edge_order;

    int connecting_distance = infinity;
    int meeting_point = -1;

    // Initialise queue.
    auto forward_comparator = [&forward_key](int left, int right) {
        return (forward_key[left]) < (forward_key[right]);
    };
    auto backward_comparator = [&backward_key](int left, int right) {
        return (backward_key[left]) < (backward_key[right]);
    };
    postorder_heap<2, int, std::vector<int>, decltype(forward_comparator)> forward_queue(forward_comparator);
    postorder_heap<2, int, std::vector<int>, decltype(backward_comparator)> backward_queue(backward_comparator);

    forward_queue.push(source);
    backward_queue.push(target);
    forward_cumulative_weight[source] = 0;
    backward_cumulative_weight[target] = 0;
    forward_key[source] = forward_heuristic_(source);
    backward_key[target] = backward_heuristic_(target);

    while (!forward_queue.empty() && !backward_queue.empty()) {
        int forward_local_source = forward_queue.top();
        forward_queue.pop();
        int backward_local_source = backward_queue.top();
        backward_queue.pop();

        if (connecting_distance < infinity && (forward_cumulative_weight[forward_local_source] + backward_cumulative_weight[backward_local_source]) >= connecting_distance + backward_heuristic_(target))
            break;

        // Visit one node in the forward search ...
        if (!forward_visited[forward_local_source]) {
            forward_visited[forward_local_source] = true;
            visits++;

            // For every out-going edge ...
            for (auto edge : graph.getOutgoing(forward_local_source)) {
                int local_target = edge.first;
                int weight = edge.second;
                int tentative_distance = forward_cumulative_weight[forward_local_source] + weight;
                forward_edge_order.emplace_back(forward_local_source, local_target);

                // If current edge allows for a shorter route -> relax
                if (tentative_distance < forward_cumulative_weight[local_target]) {
                    forward_cumulative_weight[local_target] = tentative_distance;
                    forward_key[local_target] = tentative_distance + forward_heuristic_(local_target);
                    forward_parent[local_target] = forward_local_source;
                    forward_queue.push(local_target);

                    if (backward_cumulative_weight[local_target] < infinity && tentative_distance + backward_cumulative_weight[local_target] < connecting_distance) {
                        connecting_distance = tentative_distance + backward_cumulative_weight[local_target];
                        meeting_point = local_target;
                    }
                }
            }
        }

        // Then visit one node in the backwards search ...
        if (!backward_visited[backward_local_source]) {
            backward_visited[backward_local_source] = true;
            visits++;

            // For every incoming edge ...
            for (auto edge : graph.getIncoming(backward_local_source)) {
                int local_target = edge.first;
                int weight = edge.second;
                int tentative_distance = backward_cumulative_weight[backward_local_source] + weight;
                backward_edge_order.emplace_back(backward_local_source, local_target);

                // If current edge allows for a shorter route -> relax
                if (tentative_distance < backward_cumulative_weight[local_target]) {
                    backward_cumulative_weight[local_target] = tentative_distance;
                    backward_key[local_target] = tentative_distance + backward_heuristic_(local_target);
                    backward_parent[local_target] = backward_local_source;
                    backward_queue.push(local_target);

                    if (forward_cumulative_weight[local_target] < infinity && tentative_distance + forward_cumulative_weight[local_target] < connecting_distance) {
                        connecting_distance = tentative_distance + forward_cumulative_weight[local_target];
                        meeting_point = local_target;
                    }
                }
            }
        }
    }

    return {forward_cumulative_weight, backward_cumulative_weight, forward_parent, backward_parent, meeting_point, visits, forward_edge_order, backward_edge_order};
}


std::list<int> BidirectionalAStar::extractPath(std::vector<int> forward_parent, std::vector<int> backward_parent, int meeting_point) {
    std::list<int> path;
    // If no path exists...
    if (meeting_point == -1)
        return path;
    int current = meeting_point;
    while (current != -1) {
        path.emplace_back(current);
        current = forward_parent[current];
    }
    path.reverse();
    current = backward_parent[meeting_point];
    while (current != -1) {
        path.emplace_back(current);
        current = backward_parent[current];
    }
    return path;
}