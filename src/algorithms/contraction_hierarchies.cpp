//
// Created by asha on 4/21/22.
//

#include "contraction_hierarchies.h"
ContractionHierarchies::ContractionHierarchies() : ContractionHierarchies(true, true, true, 5)
{

}

ContractionHierarchies::ContractionHierarchies(bool use_edge_diff, bool use_deleted_neighbours, bool use_search_space_size, int hop_limit) : 
use_edge_diff_(use_edge_diff),
use_deleted_(use_deleted_neighbours),
use_search_space_size_(use_search_space_size),
hop_limit_(hop_limit) 
{

}

std::vector<int>& ContractionHierarchies::getOrder() {
    return order_;
}

int ContractionHierarchies::computeKey(int vertex) {
    auto contraction_data = contract(vertex, true);
    int edge_difference = contraction_data.first;
    int search_space_size = contraction_data.second;
    int key = 0;
    if (use_edge_diff_)
        key += 190 * edge_difference;
    if (use_deleted_)
        key += 110 * contracted_neighbours_[vertex];
    if (use_search_space_size_)
        key += search_space_size;
    return key;
}

void ContractionHierarchies::updateKeys(const std::vector<Edge>& incoming, const std::vector<Edge>& outgoing) {
    for (auto edge : incoming) {
        int source = std::get<1>(edge);
        contracted_neighbours_[source]++;
    }
    for (auto edge : outgoing) {
        int target = std::get<1>(edge);
        contracted_neighbours_[target]++;
    }
}

void ContractionHierarchies::initialiseMembers(Graph& graph) {
    int no_vertices = graph.vertices();
    overlay_graph_ = Graph(graph);
    contracted_neighbours_ = std::vector<int>(no_vertices, 0);

    forward_cumulative_weight_ = fast_vector<int>(no_vertices, std::numeric_limits<int>::max());
    forward_visited_ = fast_vector<bool>(no_vertices, false);
    forward_parent_ = fast_vector<int>(no_vertices, -1);

    backward_cumulative_weight_ = fast_vector<int>(no_vertices, std::numeric_limits<int>::max());
    backward_visited_ = fast_vector<bool>(no_vertices, false);
    backward_parent_ = fast_vector<int>(no_vertices, -1);

    is_target_ = fast_vector<bool>(no_vertices, false);
    hops_ = fast_vector<int>(no_vertices, 0);
    order_ = std::vector<int>(no_vertices, -1);
}

void ContractionHierarchies::createContractionHierarchy(Graph& graph) {
    // Initialise members
    initialiseMembers(graph);

    // Initialise local variables
    int number_contracted = 0;
    int lazy_updates = 0;

    // Create queue
    auto comparator = [](Entry left, Entry right) {
        return left < right;
    };
    Queue queue(comparator);

    // Push all vertices to queue
    for (int vertex = 0; vertex < overlay_graph_.vertices(); vertex++)
        queue.push({computeKey(vertex), vertex});

    // While not all vertices have been contracted
    while (!queue.empty()) {
        // If restruction interval has been reached...
        if (number_contracted % reconstruction_interval_ == 0) {
            // Reconstruct if average lazy-updates > 1.
            if (lazy_updates / reconstruction_interval_ > 1) 
                reconstructQueue(queue);
            // Reset counter
            lazy_updates = 0;
        }
        // Get vertex which is most suited for contraction.
        Entry head = queue.top();
        queue.pop();
        // Lazy update
        auto old_key = head.first;
        auto new_key = computeKey(head.second);
        if (new_key != old_key)
            lazy_updates++;
        // Check if head is still
        head = { new_key, head.second };
        Entry next_head = (!queue.empty()) ? queue.top() : head;
        if (queue.empty() || head <= next_head) {
            int vertex =  head.second;
            contract(vertex, false);
            order_[vertex] = number_contracted;
            number_contracted++;
        }
        else {
            // If not, push back into queue.
            queue.push(head);
        }
    }
    overlay_graph_ = Graph(graph);
    constructGraph(shortcuts_);
}

std::pair<int, int> ContractionHierarchies::contract(int vertex, bool dry_run) {
    forward_cumulative_weight_.reset();
    forward_visited_.reset();
    is_target_.reset();
    hops_.reset();
    auto& outgoing = overlay_graph_.getOutgoing(vertex);
    auto& incoming = overlay_graph_.getIncoming(vertex);
    int search_space_size = 0;


    int max_outgoing_weight = 0;
    std::vector<int> targets;
    for (auto& edge : outgoing) {
        // Determine target vertices ...
        int target = edge.first;
        is_target_[target] = true;
        targets.push_back(target);

        // Determine max outgoing weight for weight limit.
        int weight = edge.second;
        max_outgoing_weight = std::max(max_outgoing_weight, weight);
    }

    // Create shortcut buffer
    std::vector<Shortcut> shortcut_buffer;

    // Determine min incoming weight ...
    int min_incoming_weight = std::numeric_limits<int>::max();
    for (auto outgoing_edge : outgoing) {
        int target = outgoing_edge.first;

        // Do one-hop backward search.
        for (auto edge : overlay_graph_.getIncoming(target)) {
            int weight = edge.second;

            // Relax weights for target
            min_incoming_weight = std::min(min_incoming_weight, weight);
        }
    }

    // For each incoming edge ...
    for (auto incoming_edge : incoming) {
        // Determine source vertex ...
        int source = incoming_edge.first;
        int incoming_weight = incoming_edge.second;
        
        // Compute weight limit of forward search ...
        int weight_limit = incoming_weight + max_outgoing_weight - min_incoming_weight;

        // Do forward-search for n - 1 hops.
        int visited_nodes = search(source, (int) targets.size(), vertex, weight_limit, hop_limit_ - 1);
        search_space_size += visited_nodes;

        // For each out going edge ...
        for (auto outgoing_edge : outgoing) {
            int target = outgoing_edge.first;

            if (target == source)
                continue;

            // Increment search space size ...    
            search_space_size += 1;
            // Compute shortcut weight. 
            int outgoing_weight = outgoing_edge.second;
            int shortcut_weight = incoming_weight + outgoing_weight;

            // Do one-hop backward search.
            const auto target_incoming = overlay_graph_.getIncoming(target);
            for (auto edge : target_incoming) {
                int incoming_source = edge.first;

                // Relax weights for target... if weight != inf
                if (forward_cumulative_weight_[incoming_source] == std::numeric_limits<int>::max())
                    continue; 

                int weight = edge.second;
                forward_cumulative_weight_[target] = std::min(forward_cumulative_weight_[target], forward_cumulative_weight_[incoming_source] + weight);
            }

            // If no witness was found ...
            if (shortcut_weight < forward_cumulative_weight_[target]) {
                // Place shortcut into buffer for later creation ...
                shortcut_buffer.emplace_back(source, vertex, target, shortcut_weight);
            }
        }
    }
    // Reset targets ...
    is_target_.reset();

    if (!dry_run) {
        // Iff not dry run, perform actual contraction.
        updateKeys(incoming, outgoing);
        removeEdges(vertex);
        createShortcuts(shortcut_buffer);
    };
    int edge_difference = (shortcut_buffer.size()) - (incoming.size() + outgoing.size());
    return {edge_difference, search_space_size};
}

void ContractionHierarchies::createShortcuts(std::vector<Shortcut>& buffer) {
    for (auto& shortcut : buffer) {
        int source = std::get<0>(shortcut);
        int target = std::get<2>(shortcut);
        int weight = std::get<3>(shortcut);

        shortcuts_[{source, target}] = shortcut;

        // Insert into overlays    
        overlay_graph_.createEdge(source, target, weight);
    }
}

void ContractionHierarchies::constructGraph(const std::map<std::pair<int, int>, Shortcut>& shortcuts) {
    for (auto entry : shortcuts) {
        auto shortcut = entry.second;
        int source = std::get<0>(shortcut);
        int target = std::get<2>(shortcut);
        int weight = std::get<3>(shortcut);
        // Insert into overlays    
        overlay_graph_.createEdge(source, target, weight);
    }
}

void ContractionHierarchies::reconstructQueue(Queue queue) {
    auto queue_container = queue.container();
    queue.clear();
    for (auto entry : queue_container) {
        entry = {computeKey(entry.second), entry.second};
        queue.push(entry);
    }
}

void ContractionHierarchies::removeEdges(int vertex) {
    // Beware : If you for some reason at three AM decide outgoing or ingoing should be references, then you are postponing bedtime until five AM.
    // Reason : Deleting from referenced list while iterating same list is stupid.
    int edges = overlay_graph_.edges();
    const auto outgoing = overlay_graph_.getOutgoing(vertex);
    for (auto edge : outgoing) {
        overlay_graph_.removeEdge(vertex, edge.first);
    }
    const auto incoming = overlay_graph_.getIncoming(vertex);
    for (auto edge : incoming) {
        overlay_graph_.removeEdge(edge.first, vertex);
    }
}

int ContractionHierarchies::search(int source, int targets, int ignore, int weight_limit, int hop_limit) {
    // Reset datastructures for next search..
    forward_cumulative_weight_.reset();
    forward_visited_.reset();
    hops_.reset();

    int settled_targets = 0;
    int visits = 0;
    
    typedef std::pair<int, int> queue_entry;
    std::priority_queue<queue_entry, std::vector<queue_entry>, std::greater<>> queue;

    forward_cumulative_weight_[source] = 0;
    queue.push({0, source});
    while (!queue.empty()) {
        int local_source = queue.top().second;
        queue.pop();

        // If search exceeds cumulative_weight limit, end search.
        if (forward_cumulative_weight_[local_source] > weight_limit)
            break;

        // If all targets have been found, end search.
        if (settled_targets == targets)
            break;

        // If would exceed hop_limit... skip
        if (hops_[local_source] == hop_limit)
            continue;

        // If already visited... skip
        if (forward_visited_[local_source]) 
            continue;

        if (is_target_[local_source])
            settled_targets++;

        // Mark visited
        forward_visited_[local_source] = true;
        visits++;
        for (auto edge : overlay_graph_.getOutgoing(local_source)) {
            int local_destination = edge.first; // Determine neighbour

            // If local_destination is ignored... skip
            if (local_destination == ignore)
                continue;

            int weight = edge.second;
            int tentative_cumulative_weight = forward_cumulative_weight_[local_source] + weight;
            int tentative_hops = hops_[local_source] + 1; 

            // If current edge allows for a shorter route -> relax
            if (tentative_cumulative_weight < forward_cumulative_weight_[local_destination]) {
                forward_cumulative_weight_[local_destination] = tentative_cumulative_weight;
                hops_[local_destination] = tentative_hops;
                queue.push({tentative_cumulative_weight, local_destination});
            }
        }
    }
    return visits;
}

bool ContractionHierarchies::stall(int local_source, std::vector<Edge>& edges, fast_vector<int>& weights) {
    int infinity = std::numeric_limits<int>::max();
    bool can_stall = false;
    for (auto& edge : edges) {
        int local_target = edge.first;
        int edge_weight = edge.second;
        if (order_[local_target] < order_[local_source]) {
            continue;
        }
        if (weights[local_target] != infinity && weights[local_target] + edge_weight < weights[local_source])
            can_stall = true;
    }
    return can_stall;
}

CHData ContractionHierarchies::query(int source, int target) {
    int visits = 0;
    int queue_insertions = 0;
    int queue_extractions = 0;

    // Determine number of nodes.
    int number_vertices = overlay_graph_.vertices();

    // Initialise observed distances with infinity.
    forward_cumulative_weight_.reset();
    forward_parent_.reset();
    forward_visited_.reset();
    
    backward_cumulative_weight_.reset();
    backward_parent_.reset();
    backward_visited_.reset();

    int infinity = std::numeric_limits<int>::max();
    int connecting_distance = infinity;
    int meeting_point = -1;

    typedef std::pair<int, int> queue_entry;
    std::priority_queue<queue_entry, std::vector<queue_entry>, std::greater<>> forward_queue;
    std::priority_queue<queue_entry, std::vector<queue_entry>, std::greater<>> backward_queue;

    std::list<std::pair<int, int>> forward_edge_order;
    std::list<std::pair<int, int>> backward_edge_order;

    forward_queue.push({0, source});
    backward_queue.push({0, target});
    forward_cumulative_weight_[source] = 0;
    backward_cumulative_weight_[target] = 0;
    bool backward_search_terminated = false;
    bool forward_search_terminated = false;
    while (!forward_search_terminated || !backward_search_terminated) {
        if (!forward_search_terminated) {
            int local_source = forward_queue.top().second;
            forward_queue.pop();
            queue_extractions++;

            if (forward_cumulative_weight_[local_source] > connecting_distance) {
                forward_search_terminated = true;
                continue;
            }

            // Visit one node in the forward search ...
            if (!forward_visited_[local_source]) {
                forward_visited_[local_source] = true;
                visits++;

                bool stalled = stall(local_source, overlay_graph_.getIncoming(local_source), forward_cumulative_weight_);

                if (!stalled) {
                    // For every out-going edge ...
                    for (auto& edge : overlay_graph_.getOutgoing(local_source)) {
                        int local_target = edge.first;
                        if (order_[local_target] < order_[local_source]) {
                            continue;
                        }
                        int weight = edge.second;
                        int tentative_distance = forward_cumulative_weight_[local_source] + weight;
                        forward_edge_order.emplace_back(local_source, local_target);
                        // If current edge allows for a shorter route -> relax
                        if (tentative_distance < forward_cumulative_weight_[local_target]) {
                            forward_cumulative_weight_[local_target] = tentative_distance;
                            forward_parent_[local_target] = local_source;
                            forward_queue.push({tentative_distance, local_target});
                            queue_insertions++;
                            if (backward_cumulative_weight_[local_target] < infinity && tentative_distance + backward_cumulative_weight_[local_target] < connecting_distance) {
                                connecting_distance = tentative_distance + backward_cumulative_weight_[local_target];
                                meeting_point = local_target;
                            }
                        }
                    }
                }
            }
            forward_search_terminated = forward_queue.empty();
        }
        if (!backward_search_terminated) {
            int local_source = backward_queue.top().second;
            backward_queue.pop();
            queue_extractions++;

            if (backward_cumulative_weight_[local_source] > connecting_distance) {
                backward_search_terminated = true;
                continue;
            }

            // Then visit one node in the backwards search ...
            if (!backward_visited_[local_source]) {
                backward_visited_[local_source] = true;
                visits++;
            
                bool stalled = stall(local_source, overlay_graph_.getOutgoing(local_source), backward_cumulative_weight_);
                if (!stalled) {
                    // For every incoming edge ...
                    for (auto& edge : overlay_graph_.getIncoming(local_source)) {
                        int local_target = edge.first;
                        if (order_[local_target] < order_[local_source]) {
                            continue;
                        }
                        int weight = edge.second;
                        int tentative_distance = backward_cumulative_weight_[local_source] + weight;
                        backward_edge_order.emplace_back(local_source, local_target);
                        // If current edge allows for a shorter route -> relax
                        if (tentative_distance < backward_cumulative_weight_[local_target]) {
                            backward_cumulative_weight_[local_target] = tentative_distance;
                            backward_parent_[local_target] = local_source;
                            backward_queue.push({tentative_distance, local_target});
                            queue_insertions++;
                            if (forward_cumulative_weight_[local_target] < infinity && tentative_distance + forward_cumulative_weight_[local_target] < connecting_distance) {
                                connecting_distance = tentative_distance + forward_cumulative_weight_[local_target];
                                meeting_point = local_target;
                            }
                        }
                    }
                }
            }
            backward_search_terminated = backward_queue.empty();            
        }
    }

    return {forward_cumulative_weight_, backward_cumulative_weight_, forward_parent_, backward_parent_, meeting_point, visits, queue_insertions, queue_extractions, std::move(forward_edge_order), std::move(backward_edge_order)};  
}

std::list<int> ContractionHierarchies::decompress(int source, int target) {
    if (!shortcuts_.contains({source, target}))
        return {};
    auto shortcut = shortcuts_[{source, target}];
    int skipped = std::get<1>(shortcut);

    auto left = decompress(source, skipped);
    auto right = decompress(skipped, target);

    left.push_back(skipped);
    left.splice(left.end(), right);
    return left;
}

std::list<int> ContractionHierarchies::decompress(std::list<int> compressed_path) {
    std::list<int> decompressed_path;
    int source = compressed_path.front();
    compressed_path.pop_front();
    int target = compressed_path.front();
    compressed_path.pop_front();
    
    while (!compressed_path.empty()) {
        auto decompressed_edge = decompress(source, target);
        decompressed_path.push_back(source);
        decompressed_path.splice(decompressed_path.end(), decompressed_edge);
        source = target;
        target = compressed_path.front();
        compressed_path.pop_front();
    }

    auto decompressed_edge = decompress(source, target);
    decompressed_path.push_back(source);
    decompressed_path.splice(decompressed_path.end(), decompressed_edge);
    decompressed_path.push_back(target);

    return decompressed_path;
}

void ContractionHierarchies::setReconstructionCheckInterval(int interval) {
    reconstruction_interval_ = interval;
}

std::list<int> ContractionHierarchies::extractPath(fast_vector<int>& forward_parent, fast_vector<int>& backward_parent, int meeting_point) {
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
    return decompress(path);
}