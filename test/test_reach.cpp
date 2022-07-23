#include <fstream>
#include <queue>
#include "gtest/gtest.h"
#include "../src/graph/graph.h"
#include "../src/algorithms/reach.h"
#include "../src/algorithms/landmark.h"


TEST(Reach, makeNoneBoundedFile) {
    std::string area = "malta";
    std::vector<int> bounds;
    std::string previous_reach_file = (std::string) TEST_RESOURCE_DIR+"/"+area+"/reach_bounds_50.txt";
    std::cout << previous_reach_file << std::endl;
    std::ifstream b_stream(previous_reach_file, std::ios::in);
    int reach;

    std::string path = (std::string) (std::string) TEST_RESOURCE_DIR+"/"+area+"/reach_bounds_0.txt";
    std::cout << path << std::endl;
    std::ofstream reach_bounds_output_stream(path, std::ios::out);

    while (b_stream >> reach) {
        reach_bounds_output_stream << INT_MAX << "\n";
    }
    reach_bounds_output_stream.close();
    b_stream.close();
}

void print_line(std::string msg) {
    std::cout << msg << std::endl;
}

/*TEST(Reach, compareRunningTimes) {
    int iterations = 100;

    std::string area_folder = (std::string) TEST_RESOURCE_DIR + "/malta";
    print_line("Lode graph from: "+area_folder);
    std::string vertices_path = area_folder + "/nodes.txt";
    std::string edges_path = area_folder + "/edges.txt";
    std::ifstream v_stream(vertices_path, std::ios::in);
    std::ifstream e_stream(edges_path, std::ios::in);
    Graph graph(v_stream, e_stream);
    v_stream.close();
    e_stream.close();

    print_line("Generate "+std::to_string(iterations)+" queries.");
    std::vector<std::pair<int, int>> queries;
    queries.reserve(iterations);
    std::mt19937 gen(0); // The none-randomness of the seed is intentional. The experiments should be reproducible.
    std::uniform_int_distribution<> uniformIntDistributionRange(0, graph.vertices()-1);
    queries.reserve(iterations);
    for (int iteration = 0; iteration < iterations; ++iteration) {
        int origin = uniformIntDistributionRange(gen);
        int destination = uniformIntDistributionRange(gen);
        queries.emplace_back(origin, destination);
    }

    print_line("Generate landmarks");
    Landmark landmark(graph);
    int landmark_count = 3;
    std::vector<int> landmark_ids = landmark.selectFarthestByEuclideanDistance(landmark_count);
    landmark.generate(landmark_ids);

    long long cumulative_time_a_star = 0;
    long long cumulative_time_landmarks = 0;

    int count_a_star = 0;
    int count_landmarks = 0;
    print_line("Run A*");
    for (int i = 0; i < queries.size(); ++i) {
        count_a_star++;
        auto query = queries[i];
        static auto euclideanDistanceHeuristic = [&graph, &query](int id) {
            return (int) std::ceil(computeWeight(
                    graph.getCoordinates(id),graph.getCoordinates(query.second)));
        };
        AStar algorithm(euclideanDistanceHeuristic);
        auto start_time = std::chrono::steady_clock::now();
        algorithm.run(graph, query.first, query.second);
        cumulative_time_a_star += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start_time).count();
        if ((i+1) % 10 == 0) {
            print_line("    Performed "+std::to_string(i+1)+" iterations");
        }
    }

    print_line("Run Landmarks");
    AStar algorithm([&landmark](int id) {
        auto landmark_bound = landmark.apply(id);
        //std::cout << "landmark bound: " << landmark_bound << ", for id: " << id << std::endl;
        return landmark_bound;
    });
    for (int i = 0; i < queries.size(); ++i) {
        count_landmarks++;
        auto query = queries[i];
        int recommended_number_of_active_landmarks = 3;
        auto start_time = std::chrono::steady_clock::now();
        std::vector<int> picked_landmarks = landmark.pick(graph.getCoordinates(query.first),
                                                          graph.getCoordinates(query.second),
                                                          recommended_number_of_active_landmarks);
        landmark.activate(picked_landmarks);
        landmark.setDestination(query.second);
        algorithm.run(graph, query.first, query.second);
        cumulative_time_landmarks += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start_time).count();
        landmark.deactivate();
        if ((i+1) % 10 == 0) {
            print_line("    Performed "+std::to_string(i+1)+" iterations");
        }
    }
    print_line("Star iterated      "+std::to_string(count_a_star)+" times");
    print_line("Landmarks iterated "+std::to_string(count_landmarks)+" times");


    print_line("AStar cumulative time:     "+std::to_string(cumulative_time_a_star));
    print_line("Landmarks cumulative time: "+std::to_string(cumulative_time_landmarks));

    print_line("AStar average time:     "+std::to_string(cumulative_time_a_star/iterations));
    print_line("Landmarks average time: "+std::to_string(cumulative_time_landmarks/iterations));
    std::cout << ";";
}*/

Graph smallTestGraph() {
    std::vector<int> vertices;
    std::vector<std::tuple<int, int, int>> edges;
    // add graph
    vertices.emplace_back(0);
    vertices.emplace_back(1);
    vertices.emplace_back(2);
    vertices.emplace_back(3);
    vertices.emplace_back(4);
    vertices.emplace_back(12);
    vertices.emplace_back(11);
    vertices.emplace_back(10);
    vertices.emplace_back(9);
    vertices.emplace_back(8);
    vertices.emplace_back(5);
    vertices.emplace_back(6);
    vertices.emplace_back(7);
    edges.emplace_back(1, 3, 20);
    edges.emplace_back(1, 4, 10);
    edges.emplace_back(1, 10, 25);
    edges.emplace_back(2, 1, 10);
    edges.emplace_back(3, 2, 12);
    edges.emplace_back(3, 8, 20);
    edges.emplace_back(4, 9, 7);
    edges.emplace_back(5, 7, 25);
    edges.emplace_back(6, 7, 15);
    edges.emplace_back(7, 5, 25);
    edges.emplace_back(7, 6, 15);
    edges.emplace_back(7, 8, 13);
    edges.emplace_back(7, 10, 40);
    edges.emplace_back(7, 12, 18);
    edges.emplace_back(8, 3, 20);
    edges.emplace_back(8, 7, 13);
    edges.emplace_back(9, 8, 14);
    edges.emplace_back(10, 1, 25);
    edges.emplace_back(10, 7, 40);
    edges.emplace_back(11, 5, 10);
    edges.emplace_back(12, 11, 14);
    Graph graph(vertices, edges);
    return graph;
}

TEST(Reach, reachBoundComputationExact) {
    Graph graph = smallTestGraph();
    std::vector<int> bounds(graph.vertices(), INT_MAX);

    std::vector<int> bound_search_limiters1{INT_MAX/3};
    std::vector<int> expected_bounds1{0, 25, 27, 39, 20, 24, 0, 49, 59, 27, 0, 14, 14};
    Reach::iterativeReachBoundComputation(graph, bound_search_limiters1, bounds);
    //std::cout << "expected_bounds.size() = " << expected_bounds1.size() << ", results.size() = " << results.size() << std::endl;
    EXPECT_EQ(expected_bounds1.size(), bounds.size());
    for (int i = 0; i < bounds.size(); ++i) {
        //std::cout << "EXPECT_EQ(expected_bounds[" << i << "] = " << expected_bounds1[i] << ", results[" << i << "] = " << results[i] << ")" << std::endl;
        //std::cout << "results[" << i << "] = " << results[i] << std::endl;
        EXPECT_EQ(expected_bounds1[i], bounds[i]);
    }
}

TEST(Reach, iterativeReachBoundComputation) {
    Graph graph = smallTestGraph();
    std::vector<int> bounds1(graph.vertices(), INT_MAX);

    std::vector<int> bound_search_limiters1{10};
    std::vector<int> expected_bounds1{0, INT_MAX, INT_MAX, INT_MAX, INT_MAX, INT_MAX, 0, INT_MAX, INT_MAX, INT_MAX, INT_MAX, INT_MAX, INT_MAX};
    Reach::iterativeReachBoundComputation(graph, bound_search_limiters1, bounds1);
    //std::cout << "expected_bounds.size() = " << expected_bounds1.size() << ", results.size() = " << results.size() << std::endl;
    EXPECT_EQ(expected_bounds1.size(), bounds1.size());
    for (int i = 0; i < bounds1.size(); ++i) {
        //std::cout << "EXPECT_EQ(expected_bounds[" << i << "] = " << expected_bounds1[i] << ", results[" << i << "] = " << results.bounds[i] << ")" << std::endl;
        EXPECT_EQ(expected_bounds1[i], bounds1[i]);
    }

    std::vector<int> bounds2(graph.vertices(), INT_MAX);
    std::vector<int> bound_search_limiters2{10, 20};
    std::vector<int> expected_bounds2{0, INT_MAX, INT_MAX, INT_MAX, INT_MAX, INT_MAX, 0, INT_MAX, INT_MAX, INT_MAX, 0, 14, 14};
    Reach::iterativeReachBoundComputation(graph, bound_search_limiters2, bounds2);
    //std::cout << "expected_bounds.size() = " << expected_bounds2.size() << ", results.size() = " << results.size() << std::endl;
    EXPECT_EQ(expected_bounds2.size(), bounds2.size());
    for (int i = 0; i < bounds2.size(); ++i) {
        //std::cout << "EXPECT_EQ(expected_bounds[" << i << "] = " << expected_bounds2[i] << ", results[" << i << "] = " << results.bounds[i] << ")" << std::endl;
        EXPECT_EQ(expected_bounds2[i], bounds2[i]);
    }

    std::vector<int> bounds3(graph.vertices(), INT_MAX);
    std::vector<int> bound_search_limiters3{10, 20, 30};
    std::vector<int> expected_bounds3{0, 25, 27, INT_MAX, 35, 24, 0, INT_MAX, INT_MAX, 42, 0, 14, 14};
    Reach::iterativeReachBoundComputation(graph, bound_search_limiters3, bounds3);
    //std::cout << "expected_bounds.size() = " << expected_bounds3.size() << ", results.size() = " << results.size() << std::endl;
    EXPECT_EQ(expected_bounds3.size(), bounds3.size());
    for (int i = 0; i < bounds3.size(); ++i) {
        //std::cout << "EXPECT_EQ(expected_bounds[" << i << "] = " << expected_bounds3[i] << ", results[" << i << "] = " << results.bounds[i] << ")" << std::endl;
        EXPECT_EQ(expected_bounds3[i], bounds3[i]);
    }
}

TEST(Reach, compareIterativeToRepeatedSingleComputation) {
    std::ifstream v_stream((std::string) TEST_RESOURCE_DIR + "/bornholm/nodes.txt", std::ios::in);
    std::ifstream e_stream((std::string) TEST_RESOURCE_DIR + "/bornholm/edges.txt", std::ios::in);
    std::cout<<"Load graph."<<std::endl;
    Graph graph = Graph(v_stream, e_stream);

    std::vector<int> bound_search_limiters {50, 100, 150, 250};
    std::vector<int> iterative_computed_bounds(graph.vertices(), INT_MAX);
    std::cout<<"Perform reach bound computation."<<std::endl;
    Reach::iterativeReachBoundComputation(graph, bound_search_limiters, iterative_computed_bounds);

    std::vector<int> single_computed_bounds(graph.vertices(), INT_MAX);
    for (int bound_search_limiter : bound_search_limiters) {
        Reach::singleReachBoundComputation(graph, bound_search_limiter, single_computed_bounds);
    }
    for (int i = 0; i < graph.vertices(); ++i) {
        EXPECT_EQ(iterative_computed_bounds[i], single_computed_bounds[i]);
    }
}

TEST(Reach, dijkstra) {
    int root = 1;
    Graph graph = smallTestGraph();

    int infinity = std::numeric_limits<int>::max();
    fast_vector<int> distances(graph.vertices(), infinity);
    fast_vector<int> parents(graph.vertices(), -1);
    fast_vector<int> visited_in_order(graph.vertices(), -1);
    fast_vector<bool> visited(graph.vertices(), false);
    std::vector<int> first_edge_length(graph.vertices(), 0);
    std::vector<bool> is_bounded(graph.vertices(), false);
    Reach::dijkstra(distances, parents, visited, visited_in_order, first_edge_length, graph, is_bounded, root, INT_MAX / 2 - 10000);

    std::map<int, int> expected_parents;
    expected_parents[10] = 1;
    expected_parents[11] = 12;
    expected_parents[12] = 7;
    expected_parents[5] = 7;
    expected_parents[6] = 7;
    expected_parents[7] = 8;
    expected_parents[8] = 9;
    expected_parents[9] = 4;
    expected_parents[4] = 1;
    expected_parents[2] = 3;
    expected_parents[3] = 1;
    for (auto[vertex, parent]: expected_parents) {
        EXPECT_EQ(parents[vertex], parent);
    }

    std::map<int, int> expected_distances;
    expected_distances[0] = std::numeric_limits<int>::max();
    expected_distances[1] = 0;
    expected_distances[2] = 32;
    expected_distances[3] = 20;
    expected_distances[4] = 10;
    expected_distances[5] = 69;
    expected_distances[6] = 59;
    expected_distances[7] = 44;
    expected_distances[8] = 31;
    expected_distances[9] = 17;
    expected_distances[10] = 25;
    expected_distances[11] = 76;
    expected_distances[12] = 62;
    for (int i = 0; i < expected_distances.size(); ++i) {
        EXPECT_EQ(expected_distances[i], distances[i]);
    }

    std::map<int, std::set<int>> expected_leafs;
    expected_leafs[1].insert(2);
    expected_leafs[1].insert(6);
    expected_leafs[1].insert(5);
    expected_leafs[1].insert(11);
    expected_leafs[1].insert(10);
    expected_leafs[3].insert(2);
    expected_leafs[2].insert(2);
    expected_leafs[4].insert(6);
    expected_leafs[4].insert(5);
    expected_leafs[4].insert(11);
    expected_leafs[9].insert(6);
    expected_leafs[9].insert(5);
    expected_leafs[9].insert(11);
    expected_leafs[8].insert(6);
    expected_leafs[8].insert(5);
    expected_leafs[8].insert(11);
    expected_leafs[7].insert(6);
    expected_leafs[7].insert(5);
    expected_leafs[7].insert(11);
    expected_leafs[6].insert(6);
    expected_leafs[5].insert(5);
    expected_leafs[12].insert(11);
    expected_leafs[11].insert(11);
    expected_leafs[10].insert(10);


    // Allocate children_counts once (all is made zero again doing every execution, as part of the essential algorithm
    std::vector<int> children_counts(graph.vertices());
    std::vector<std::list<int>> leafs(graph.vertices());
    std::queue<int> queue;

    // make list of parents
    // Count number of children per parent
    // Iterate children (initialized to 1 to jump over source)
    for (int i = 1; i < graph.vertices() && visited_in_order[i] != -1; i++) {
        int vertex = visited_in_order[i];
        int parent = parents[vertex];
        children_counts[parent]++;
    }
    // Add children which themselves have zero children to queue. Also initialize the leaf lists of leafs
    // Iterate all visited vertices
    for (int i = 0; i < graph.vertices() && visited_in_order[i] != -1; i++) {
        int vertex = visited_in_order[i];
        if (children_counts[vertex] == 0) {
            queue.push(vertex);
            auto &child_leafs = leafs[vertex];
            child_leafs.insert(child_leafs.end(), vertex);
        }
    }

    while (!queue.empty()) {
        int vertex = queue.front(); queue.pop();
        std::list<int> &vertex_leafs = leafs[vertex];
        //std::cout << vertex << std::endl;
        // Do stuff

        EXPECT_EQ(expected_leafs[vertex].size(), vertex_leafs.size());
        for (int leaf : vertex_leafs) {
            EXPECT_TRUE(expected_leafs[vertex].contains(leaf));
        }

        // Done stuff
        if (vertex != root) {
            int parent = parents[vertex];
            int &children_count = children_counts[parent];
            children_count--; // Adjust child count of parent
            auto &parent_leafs = leafs[parent];
            parent_leafs.splice(parent_leafs.end(), vertex_leafs); // Add own leafs to parent leafs
            if (children_count == 0) {
                queue.push(parent);
            }
        }
    }
}
