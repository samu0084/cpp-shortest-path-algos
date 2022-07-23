//
// Created by asha on 4/21/22.
//

#ifndef ATB_CONTRACTION_HIERARCHIES_H
#define ATB_CONTRACTION_HIERARCHIES_H


#include "../graph/graph.h"
#include "../../modules/dary-heap/src/dary_heap.h"
#include <queue>
#include "fast_vector.hpp"
#include <map>
#include <list>

typedef std::tuple<int, int, int, int> Shortcut;
typedef std::tuple<fast_vector<int>&, fast_vector<int>&, fast_vector<int>&, fast_vector<int>&, int, int, int, int, std::list<std::pair<int, int>>, std::list<std::pair<int, int>>> CHData;

class ContractionHierarchies {
public:
    typedef std::pair<int, int> Entry; 
    typedef bool Comparator(Entry, Entry);
    typedef dary_heap<3, Entry, std::vector<Entry>, std::function<Comparator>> Queue;

    ContractionHierarchies();
    ContractionHierarchies(bool use_edge_diff, bool use_deleted_neighbours, bool use_search_space_size, int hop_limit);

    void createContractionHierarchy(Graph& graph);
    CHData query(int source, int target);

    std::list<int> extractPath(fast_vector<int>& forward_parent, fast_vector<int>& backward_parent, int meeting_point);

    std::vector<int>& getOrder();
    void setReconstructionCheckInterval(int interval);
private:
    std::list<int> decompress(int source, int target);
    std::list<int> decompress(std::list<int> compressedPath);
    std::pair<int, int> contract(int target, bool dry_run);
    int computeKey(int vertex);
    void updateKeys(const std::vector<Edge>& incoming, const std::vector<Edge>& outgoing);
    void createShortcuts(std::vector<Shortcut>& buffer);
    void constructGraph(const std::map<std::pair<int, int>, Shortcut>& shortcuts);
    int search(int source, int targets, int ignore, int weight_limit, int hop_limit);

    bool stall(int local_source, std::vector<Edge>& edges, fast_vector<int>& weights);

    void reconstructQueue(Queue queue);
    void removeEdges(int vertex);
    void initialiseMembers(Graph& graph);

    
    
    Graph overlay_graph_;
    int reconstruction_interval_ = 1000000;
    int hop_limit_;
    bool use_edge_diff_;
    bool use_deleted_;
    bool use_search_space_size_;
    std::map<std::pair<int, int>, Shortcut> shortcuts_;
    std::vector<int> contracted_neighbours_;
    std::vector<int> order_;

    fast_vector<int> forward_cumulative_weight_;
    fast_vector<bool> forward_visited_;
    fast_vector<int> forward_parent_;

    fast_vector<int> backward_cumulative_weight_;
    fast_vector<bool> backward_visited_;
    fast_vector<int> backward_parent_;

    fast_vector<bool> is_target_;
    fast_vector<int> hops_;
};


#endif //ATB_CONTRACTION_HIERARCHIES_H
