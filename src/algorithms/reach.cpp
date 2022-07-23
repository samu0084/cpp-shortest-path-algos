//
// Created by sx5 on 5/5/2022.
//

#include <fstream>
#include <chrono>
#include <queue>
#include "reach.h"

Reach::Reach(std::vector<int> &reaches) : reaches_(reaches) { }

std::vector<int> &Reach::read(std::istream &reach_stream, std::vector<int> &empty_read_reaches_container) {
    empty_read_reaches_container.clear();
    std::vector<int> &reaches = empty_read_reaches_container;
    int reach;
    // Load reaches ...
    while (reach_stream >> reach) {
        reaches.emplace_back(reach);
    }
    return reaches;
}

void Reach::write(std::string &output_path, std::string &file_postfix, std::vector<int> &bounds) {
    std::string path = (std::string) output_path + "/reach_bounds_" + file_postfix + ".txt";
    std::ofstream reach_bounds_output_stream(path, std::ios::out);
    for (int reach_bound : bounds) {
        reach_bounds_output_stream << reach_bound << "\n";
    }
    reach_bounds_output_stream.close();
}

bool Reach::test(int vertex, int distance_origin, int distance_destination) {
    return reaches_[vertex] >= distance_origin || reaches_[vertex] >= distance_destination;
}

bool Reach::test(int vertex, int distance_origin, int destination, Graph &graph) {
    int distance_destination = (int) computeWeight(
            graph.getCoordinates(vertex),
            graph.getCoordinates(destination));
    return reaches_[vertex] >= distance_origin || reaches_[vertex] >= distance_destination;
}

std::vector<int> &Reach::iterativeReachBoundComputation(Graph &graph, std::vector<int> &bound_search_limiters, std::vector<int> &initial_bounds) {
    int infinity = std::numeric_limits<int>::max();
    std::vector<bool> is_bounded(graph.vertices());
    for (int i = 0; i < graph.vertices(); ++i) {
        if (initial_bounds[i] != infinity) {
            is_bounded[i] = true;
        }
    }
    std::vector<int> &bounds = initial_bounds;
    for (int bound_search_limiter : bound_search_limiters) {
        iterate(graph, is_bounded, bounds, bound_search_limiter);
        for (int i = 0; i < graph.vertices(); ++i) {
            if (bounds[i] != infinity) {
                is_bounded[i] = true;
            }
        }
    }
    return bounds;
}

std::vector<int> &Reach::singleReachBoundComputation(Graph &graph, int bound_search_limiter, std::vector<int> &initial_bounds) {
    int infinity = std::numeric_limits<int>::max();
    std::vector<bool> is_bounded(graph.vertices());
    for (int i = 0; i < graph.vertices(); ++i) {
        if (initial_bounds[i] != infinity) {
            is_bounded[i] = true;
        }
    }
    std::vector<int> &bounds = initial_bounds;
    iterate(graph, is_bounded, bounds, bound_search_limiter);
    return bounds;
}

int Reach::maxKnownBound(std::vector<int> &bounds) {
    int max_found_bound = 0;
    for (int bound : bounds) {
        if (max_found_bound < bound && bound < std::numeric_limits<int>::max()) {
            max_found_bound = bound;
        }
    }
    return max_found_bound;
}

void Reach::iterate(Graph &graph,
                    std::vector<bool> &is_bounded,
                    std::vector<int> &bounds,
                    int bound_search_limiter) {
    int infinity = std::numeric_limits<int>::max();
    int treated_vertices = 0; // Only used to follow the progress doing computation

    int max_known_bound = maxKnownBound(bounds); // Note that this needs to be before some bounds are set to zero
    for (int vertex = 0; vertex < graph.vertices(); ++vertex) {
        if (!is_bounded[vertex]) {
            bounds[vertex] = 0;
        }
    }
    std::vector<int> greatest_reach(graph.vertices(), 0);
    fast_vector<int> distances(graph.vertices(), infinity);
    fast_vector<int> parents(graph.vertices(), -1);
    fast_vector<int> visited_in_order(graph.vertices(), -1); // Contains the ids in the order in which they are visited.
    fast_vector<bool> visited(graph.vertices(), false); // Entry visited[id] = true if id has been visited
    std::vector<int> first_edge_length(graph.vertices(), 0);
    std::vector<int> children_counts(graph.vertices(), 0);
    std::vector<std::list<int>> leafs(graph.vertices());
    std::queue<int> queue;

    for (int source = 0; source < graph.vertices(); ++source) {
        if (is_bounded[source]) {
            continue;
        }
        treated_vertices++;
        int max_ingoing_bound = 0;
        int max_ingoing_edge_length = 0;
        for (auto [local_origin, weight] : graph.getIncoming(source)) {
            if (is_bounded[local_origin]) {
                max_ingoing_bound = std::max(bounds[local_origin] + weight, max_ingoing_bound);
                max_ingoing_edge_length = std::max(weight, max_ingoing_edge_length);
            }
        }
        dijkstra(distances, parents, visited, visited_in_order, first_edge_length, graph, is_bounded, source,
                 2 * bound_search_limiter + max_known_bound + max_ingoing_edge_length);
        // Iterate children to count number of children per parent; start at 1 as source cannot be a child
        for (int i = 1; i < graph.vertices() && visited_in_order[i] != -1; i++) {
            int vertex = visited_in_order[i];
            int parent = parents[vertex];
            children_counts[parent]++;
        }
        // Iterate all vertices, initializing the leaf list of vertices which have zero children, and adding them to queue
        for (int i = 0; i < graph.vertices() && visited_in_order[i] != -1; i++) {
            int vertex = visited_in_order[i];
            if (children_counts[vertex] == 0) {
                auto &child_leafs = leafs[vertex];
                child_leafs.insert(child_leafs.end(), vertex);
                queue.push(vertex);
            }
        }
        // Go through path tree. Any child is visited before the parent. No further order is guaranteed.
        while (!queue.empty()) {
            int vertex = queue.front();
            queue.pop();
            std::list<int> &descendant_leafs = leafs[vertex];
            int reach = reachInTree(vertex, distances, descendant_leafs);
            improveReachBound(vertex, distances, descendant_leafs, is_bounded, bounds, max_ingoing_bound);
            if (greatest_reach[vertex] < reach)
                greatest_reach[vertex] = reach;
            if (vertex != source) {
                int parent = parents[vertex];
                int &children_count = children_counts[parent];
                children_count--; // Adjust child count of parent
                auto &parent_leafs = leafs[parent];
                parent_leafs.splice(parent_leafs.end(), descendant_leafs); // Add own leafs to parent leafs
                if (children_count == 0) { // parent child-count == zero means all leafs have been added to parent
                    queue.push(parent);
                }
            }
        }
        distances.reset();
        visited.reset();
        children_counts[source] = 0; // All other entries have been reset to zero as part of the traversal
        visited_in_order.reset();
        parents.reset();
        leafs[source].clear(); // As splice is used to move leafs until all leafs are at source, it is sufficient to reset source.
        if (treated_vertices % 10000 == 0)
            std::cout << "treated " << treated_vertices << " vertices" << std::endl;
    }
    std::cout << "treated " << treated_vertices << " vertices" << std::endl;

    for (int vertex = 0; vertex < graph.vertices(); ++vertex) {
        if (is_bounded[vertex]) {
            continue;
        }
        if (greatest_reach[vertex] >= bound_search_limiter) {
            bounds[vertex] = infinity;
        }
    }
}

int Reach::reachInTree(int vertex, fast_vector<int> &distances, const std::list<int> &descendant_leafs) {
    int reach = 0;
    int source_to_vertex_distance = distances[vertex];
    for (int leaf : descendant_leafs) {
        int vertex_to_leaf_distance = distances[leaf] - source_to_vertex_distance;
        reach = std::max(reach, std::min(source_to_vertex_distance, vertex_to_leaf_distance));
    }
    return reach;
}

void Reach::improveReachBound(int vertex, fast_vector<int> &distances, std::list<int> &descendant_leafs, std::vector<bool> &is_bounded,
                              std::vector<int> &bounds, int max_ingoing_bound) {
    for (int leaf : descendant_leafs) {
        int leaf_bound = is_bounded[leaf] ? bounds[leaf] : 0;
        int vertex_bound = std::min(max_ingoing_bound + distances[vertex], leaf_bound + distances[leaf] - distances[vertex]);
        if (bounds[vertex] < vertex_bound)
            bounds[vertex] = vertex_bound;
    }
}

void Reach::dijkstra(fast_vector<int> &distances, fast_vector<int> &parents, fast_vector<bool> &visited, fast_vector<int> &visited_in_order,
                     std::vector<int> &first_edge_length, Graph &graph, const std::vector<bool>& is_bounded, int origin, int search_limiter) {
    int visited_it_order_count = 0;
    first_edge_length[origin] = 0;
    // Initialise queue compare using 'key' -> distances from origin
    auto comparator = [&distances](int left, int right) {
        return (distances[left]) < (distances[right]);
    };
    postorder_heap<3, int, std::vector<int>, decltype(comparator)> queue(comparator);
    distances[origin] = 0;
    queue.push(origin);
    while (!queue.empty()) {
        int local_origin = queue.top();
        queue.pop();
        if (visited[local_origin]) {
            continue;
        }
        visited[local_origin] = true;
        visited_in_order[visited_it_order_count] = local_origin;
        visited_it_order_count++;
        if (is_bounded[local_origin]) {
            continue;
        }
        for (auto& edge : graph.getOutgoing(local_origin)) {
            int local_destination = edge.first;
            int last_edge_length = edge.second;
            int tentative_distance = distances[local_origin] + last_edge_length;
            int current_distance = distances[local_destination];
            if (tentative_distance < current_distance &&
                    tentative_distance < search_limiter + first_edge_length[local_origin] + last_edge_length) {

                distances[local_destination] = tentative_distance;
                first_edge_length[local_destination] = local_origin == origin ? last_edge_length : first_edge_length[local_origin];
                parents[local_destination] = local_origin;
                queue.push(local_destination);
            }
        }
    }
}
