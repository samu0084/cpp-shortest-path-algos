#include <string>
#include <iostream>
#include <fstream>
#include <queue>
#include <random>
#include <tuple>
#include <filesystem>
#include "../graph/graph.h"
#include "../algorithms/a_star.h"
#include "../algorithms/a_star_queue_safe.hpp"
#include "../algorithms/reach.h"
#include "../algorithms/a_star_reach.h"
#include "../algorithms/landmark.h"
#include "../algorithms/bidirectional_a_star.h"
#include "../algorithms/contraction_hierarchies.h"

void print_line(std::string msg) {
    std::cout << msg << '\n';
}

void write(std::vector<std::tuple<unsigned long long, int, long>> &results, std::string &complete_file_path) {
    std::ofstream output_file(complete_file_path, std::ofstream::trunc);
    output_file << "rank,visits,instructions,page faults,cache references,cache misses,branches,branch misses,microseconds\n";
    for (auto entry : results) {
        auto rank = std::get<0>(entry);
        auto visits = std::get<1>(entry);
        auto microseconds = std::get<2>(entry);
        output_file << rank
                    << ',' << visits
                    << ',' << microseconds
                    << '\n';
    }
    output_file.close();
}

void write(std::vector<std::tuple<unsigned long long, int, long, int>> &results, std::string &complete_file_path) {
    std::ofstream output_file(complete_file_path, std::ofstream::trunc);
    output_file << "rank,visits,microseconds,path_length\n";
    for (auto entry : results) {
        auto rank = std::get<0>(entry);
        auto visits = std::get<1>(entry);
        auto microseconds = std::get<2>(entry);
        auto path_length = std::get<3>(entry);
        output_file << rank
                    << ',' << visits
                    << ',' << microseconds
                    << ',' << path_length
                    << '\n';
    }
    output_file.close();
}

std::tuple<unsigned long long, int, long> timeContractionHierarchies(int source, int target, ContractionHierarchies &algorithm) {
    auto start = std::chrono::steady_clock::now();
    auto data = algorithm.query(source, target);
    auto time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count();
    auto forward_parent = std::get<2>(data);
    auto backward_parent = std::get<3>(data);
    int meeting_point = std::get<4>(data);
    int visits = std::get<5>(data);
    unsigned long long rank = algorithm.extractPath(forward_parent, backward_parent, meeting_point).size();
    return {rank, visits, time};
}

std::tuple<unsigned long long, int, long> timeAStar(int source, int target, AStar &algorithm, Graph &graph) {
    auto start = std::chrono::steady_clock::now();
    auto data = algorithm.run(graph, source, target);
    auto time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count();
    auto parents = std::get<1>(data);
    unsigned long long rank = AStar::extractPath(parents, target).size();
    int visits = std::get<2>(data);
    return {rank, visits, time};
}

std::tuple<unsigned long long, int, long> timeAStarReachWithExternal(int source, int target, AStarReach &algorithm, Graph &graph) {
    auto start = std::chrono::steady_clock::now();
    auto data = algorithm.run(graph, source, target);
    auto time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count();
    auto parents = std::get<1>(data);
    unsigned long long rank = AStarReach::extractPath(parents, target).size();
    int visits = std::get<2>(data);
    return {rank, visits, time};
}

std::tuple<unsigned long long, int, long, int> timeAStarReachLandmarkOnRun(Landmark &landmark, int number_of_landmarks_to_pick, int source, int target, AStarReach &algorithm, Graph &graph) {
    auto start = std::chrono::steady_clock::now();
    auto picked_landmarks = landmark.pick(
                graph.getCoordinates(source),
                graph.getCoordinates(target),
                number_of_landmarks_to_pick);
    landmark.activate(picked_landmarks);
    landmark.setDestination(target);
    auto data = algorithm.run(graph, source, target);
    auto time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count();
    landmark.deactivate();
    auto parents = std::get<1>(data);
    unsigned long long rank = AStarReach::extractPath(parents, target).size();
    int visits = std::get<2>(data);
    int path_length = std::get<0>(data)[target];
    return {rank, visits, time, path_length};
}



std::tuple<unsigned long long, int, long> timeLandmarks(Landmark &landmark, int number_of_landmarks_to_pick, int source, int target, AStar &algorithm, Graph &graph) {
    int average_time = 0;
    auto start = std::chrono::steady_clock::now();
    auto picked_landmarks = landmark.pick(
                graph.getCoordinates(source),
                graph.getCoordinates(target),
                number_of_landmarks_to_pick);
    landmark.activate(picked_landmarks);
    landmark.setDestination(target);
    auto data = algorithm.run(graph, source, target);
    auto time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count();
    landmark.deactivate();
    auto parents = std::get<1>(data);
    unsigned long long rank = AStar::extractPath(parents, target).size();
    int visits = std::get<2>(data);
    return {rank, visits, time};
}

std::tuple<unsigned long long, int, long> timeBidirectionalLandmarks(Landmark &landmark, int number_of_landmarks_to_pick, int source, int target, BidirectionalAStar &algorithm, Graph &graph) {
    auto start = std::chrono::steady_clock::now();
    auto picked_landmarks = landmark.pick(
            graph.getCoordinates(source),
            graph.getCoordinates(target),
            number_of_landmarks_to_pick);
    landmark.activate(picked_landmarks);
    auto data = algorithm.run(graph, source, target);
    auto time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count();
    auto forward_parent = std::get<2>(data);
    auto backward_parent = std::get<3>(data);
    int meeting_point = std::get<4>(data);
    int visits = std::get<5>(data);
    unsigned long long rank = algorithm.extractPath(forward_parent, backward_parent, meeting_point).size();
    return {rank, visits, time};
}

std::tuple<unsigned long long, int, long> timeBidirectionalAStar(int source, int target, BidirectionalAStar &algorithm, Graph &graph) {
    auto start = std::chrono::steady_clock::now();
    auto data = algorithm.run(graph, source, target);
    auto time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count();
    auto forward_parent = std::get<2>(data);
    auto backward_parent = std::get<3>(data);
    int meeting_point = std::get<4>(data);
    int visits = std::get<5>(data);
    unsigned long long rank = algorithm.extractPath(forward_parent, backward_parent, meeting_point).size();
    return {rank, visits, time};
}

void experimentsLandmarks(Landmark &landmark, int number_of_landmarks_to_pick, std::string &area_folder, std::vector<std::pair<int, int>> &queries, Graph &graph) {
    std::vector<std::tuple<unsigned long long, int, long>> results;
    results.reserve(queries.size());
    static auto landmarkHeuristic = [&landmark](int id) {
        return (int) landmark.apply(id);
    };
    AStar a_star(landmarkHeuristic);
    for (int iteration = 0; iteration < queries.size(); ++iteration) {
        auto query = queries[iteration];
        auto data = timeLandmarks(landmark, number_of_landmarks_to_pick, query.first, query.second, a_star, graph);
        results.emplace_back(std::get<0>(data), std::get<1>(data), std::get<2>(data));
        if (iteration % 1000 == 0) {
            print_line("\t\tPerformed "+std::to_string(iteration)+" queries");
        }
    }
    print_line("\t\tPerformed all queries");
    std::string complete_file_path = area_folder+"compareLandmarks"+std::to_string(number_of_landmarks_to_pick)+".txt";
    write(results, complete_file_path);
}

void experimentsReachWithLandmarks(Reach &reach, Landmark &landmark, int number_of_landmarks_to_pick, std::string &area_folder, std::vector<std::pair<int, int>> &queries, Graph &graph) {
    std::vector<std::tuple<unsigned long long, int, long, int>> results;
    results.reserve(queries.size());
    for (int iteration = 0; iteration < queries.size(); ++iteration) {
        auto query = queries[iteration];
        static auto landmarkHeuristic = [&landmark](int id) {
            return (int) landmark.apply(id);
        };
        AStarReach a_star_reach(landmarkHeuristic, reach);
        auto data = timeAStarReachLandmarkOnRun(landmark, number_of_landmarks_to_pick, query.first, query.second, a_star_reach, graph);
        results.emplace_back(std::get<0>(data), std::get<1>(data), std::get<2>(data), std::get<3>(data));
        if (iteration % 10000 == 0) {
            print_line("\t\tPerformed "+std::to_string(iteration)+" queries");
        }
    }
    print_line("\t\tPerformed all queries");
    std::string complete_file_path = area_folder+"compareReachWithLandmarksContainsDistances.txt";
    write(results, complete_file_path);
}

void experimentsReachWithAStar(Reach &reach, std::string &area_folder, std::vector<std::pair<int, int>> &queries, Graph &graph) {
    std::vector<std::tuple<unsigned long long, int, long>> results;
    results.reserve(queries.size());
    for (int iteration = 0; iteration < queries.size(); ++iteration) {
        auto query = queries[iteration];
        static auto euclideanDistanceHeuristic = [&graph, &query](int id) {
            return (int) std::ceil(computeWeight(
                    graph.getCoordinates(id),graph.getCoordinates(query.second)));
        };
        AStarReach a_star_reach(euclideanDistanceHeuristic, reach);
        auto data = timeAStarReachWithExternal(query.first, query.second, a_star_reach, graph);
        results.emplace_back(std::get<0>(data), std::get<1>(data), std::get<2>(data));
        if (iteration % 10000 == 0) {
            print_line("\t\tPerformed "+std::to_string(iteration)+" queries");
        }
    }
    print_line("\t\tPerformed all queries");
    std::string complete_file_path = area_folder+"compareReachWithAStar.txt";
    write(results, complete_file_path);
}

void experimentsAStar(std::string &area_folder, std::vector<std::pair<int, int>> &queries, Graph &graph) {
    std::vector<std::tuple<unsigned long long, int, long>> results;
    results.reserve(queries.size());
    for (int iteration = 0; iteration < queries.size(); ++iteration) {
        auto query = queries[iteration];
        static auto euclideanDistanceHeuristic = [&graph, &query](int id) {
            return (int) std::ceil(computeWeight(
                    graph.getCoordinates(id),graph.getCoordinates(query.second)));
        };
        AStar a_star(euclideanDistanceHeuristic);
        auto data = timeAStar(query.first, query.second, a_star, graph);
        results.emplace_back(std::get<0>(data), std::get<1>(data), std::get<2>(data));
        if (iteration % 10000 == 0) {
            print_line("\t\tPerformed "+std::to_string(iteration)+" queries");
        }
    }
    print_line("\t\tPerformed all queries");
    std::string complete_file_path = area_folder+"compareAStar.txt";
    write(results, complete_file_path);
}

void experimentsDijkstra(std::string &area_folder, std::vector<std::pair<int, int>> &queries, Graph &graph) {
    static auto zeroHeuristic = [](int id) { return 0; };
    AStar dijkstra(zeroHeuristic);
    std::vector<std::tuple<unsigned long long, int, long>> results;
    results.reserve(queries.size());
    for (int iteration = 0; iteration < queries.size(); ++iteration) {
        auto query = queries[iteration];
        auto data = timeAStar(query.first, query.second, dijkstra, graph);
        results.emplace_back(std::get<0>(data), std::get<1>(data), std::get<2>(data));
        if (iteration % 10000 == 0) {
            print_line("\t\tPerformed "+std::to_string(iteration)+" queries");
        }
    }
    print_line("\t\tPerformed all queries");
    std::string complete_file_path = area_folder+"compareDijkstra.txt";
    write(results, complete_file_path);
}

void experimentsBidirectionalLandmarks(Landmark &landmark, int number_of_landmarks_to_pick, std::string &area_folder, std::vector<std::pair<int, int>> &queries, Graph &graph) {
    std::vector<std::tuple<unsigned long long, int, long>> results;
    results.reserve(queries.size());
    for (int iteration = 0; iteration < queries.size(); ++iteration) {
        auto query = queries[iteration];
        static auto f_heuristic = [&landmark, &query](int id) {
            landmark.setDestination(query.second);
            return (int) landmark.apply(id);
        };
        static auto r_heuristic = [&landmark, &query](int id) {
            landmark.setDestination(query.first);
            return (int) landmark.apply(id);
        };
        BidirectionalAStar bidirectional_a_star(f_heuristic, r_heuristic);
        auto data = timeBidirectionalLandmarks(landmark, number_of_landmarks_to_pick, query.first, query.second, bidirectional_a_star, graph);
        results.emplace_back(std::get<0>(data), std::get<1>(data), std::get<2>(data));
        if (iteration % 10000 == 0) {
            print_line("\t\tPerformed "+std::to_string(iteration)+" queries");
        }
    }
    print_line("\t\tPerformed all queries");
    std::string complete_file_path = area_folder+"compareBidirectionalLandmarks.txt";
    write(results, complete_file_path);
}

void experimentsBidirectionalAStar(std::string &area_folder, std::vector<std::pair<int, int>> &queries, Graph &graph) {
    std::vector<std::tuple<unsigned long long, int, long>> results;
    results.reserve(queries.size());
    for (int iteration = 0; iteration < queries.size(); ++iteration) {
        auto query = queries[iteration];
        static auto f_heuristic = [&graph, &query](int id) {
            return (int) std::ceil(computeWeight(graph.getCoordinates(id), graph.getCoordinates(query.second)));
        };
        static auto r_heuristic = [&graph, &query](int id) {
            return (int) std::ceil(computeWeight(graph.getCoordinates(id), graph.getCoordinates(query.first)));
        };
        BidirectionalAStar bidirectional_a_star(f_heuristic, r_heuristic);
        auto data = timeBidirectionalAStar(query.first, query.second, bidirectional_a_star, graph);
        results.emplace_back(std::get<0>(data), std::get<1>(data), std::get<2>(data));
        if (iteration % 10000 == 0) {
            print_line("\t\tPerformed "+std::to_string(iteration)+" queries");
        }
    }
    print_line("\t\tPerformed all queries");
    std::string complete_file_path = area_folder+"compareBidirectionalAStar.txt";
    write(results, complete_file_path);
}

void experimentsBidirectionalDijkstra(std::string &area_folder, std::vector<std::pair<int, int>> &queries, Graph &graph) {
    static auto zeroHeuristic = [](int id) { return 0; };
    BidirectionalAStar bidirectional_dijkstra(zeroHeuristic, zeroHeuristic);
    std::vector<std::tuple<unsigned long long, int, long>> results;
    results.reserve(queries.size());
    for (int iteration = 0; iteration < queries.size(); ++iteration) {
        auto query = queries[iteration];
        auto data = timeBidirectionalAStar(query.first, query.second, bidirectional_dijkstra, graph);
        results.emplace_back(std::get<0>(data), std::get<1>(data), std::get<2>(data));
        if (iteration % 10000 == 0) {
            print_line("\t\tPerformed "+std::to_string(iteration)+" queries");
        }
    }
    print_line("\t\tPerformed all queries");
    std::string complete_file_path = area_folder+"compareBidirectionalDijkstra.txt";
    write(results, complete_file_path);
}

void experimentsContractionHierarchiesEDS5(std::string &area_folder, std::vector<std::pair<int, int>> &queries, Graph &graph) {
    ContractionHierarchies algorithm(true, true, true, 5);
    algorithm.createContractionHierarchy(graph);
    std::vector<std::tuple<unsigned long long, int, long>> results;
    results.reserve(queries.size());
    for (int iteration = 0; iteration < queries.size(); ++iteration) {
        auto query = queries[iteration];
        auto data = timeContractionHierarchies(query.first, query.second, algorithm);
        results.emplace_back(std::get<0>(data), std::get<1>(data), std::get<2>(data));
        if (iteration % 10000 == 0) {
            print_line("\t\tPerformed "+std::to_string(iteration)+" queries");
        }
    }
    print_line("\t\tPerformed all queries");
    std::string complete_file_path = area_folder+"compareContractionHierarchiesEDS5.txt";
    write(results, complete_file_path);
}

void runOnAllExperiments(std::string &area_folder, int iterations, int landmark_select_count, int landmark_pick_count) {
    print_line("Lode graph from: "+area_folder);
    std::string vertices_path = area_folder + "/nodes.txt";
    std::string edges_path = area_folder + "/edges.txt";
    std::ifstream v_stream(vertices_path, std::ios::in);
    std::ifstream e_stream(edges_path, std::ios::in);
    Graph graph(v_stream, e_stream);
    v_stream.close();
    e_stream.close();

    print_line("Generate landmarks");
    Landmark landmark(graph);
    std::vector<int> landmark_ids = landmark.selectFarthestByEuclideanDistance(landmark_select_count);
    landmark.generate(landmark_ids);

    print_line("Loading reach-bounds");
    std::string reach_bounds_path = area_folder+"/reach_bounds.txt";
    std::ifstream reach_stream(reach_bounds_path, std::ios::in);
    std::vector<int> reach_bounds;
    Reach::read(reach_stream, reach_bounds);
    reach_stream.close();
    Reach reach(reach_bounds);

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

    print_line("Run queries on");
    print_line("\tContraction Hierarchies (The EDS5 variant)");
    experimentsContractionHierarchiesEDS5(area_folder, queries, graph);
    print_line("\tReach-based Routing with Landmarks");
    experimentsReachWithLandmarks(reach, landmark, landmark_pick_count, area_folder, queries, graph);
    print_line("\tReach-based Routing with A*");
    experimentsReachWithAStar(reach, area_folder, queries, graph);
    print_line("\tLandmarks");
    experimentsLandmarks(landmark, landmark_pick_count, area_folder, queries, graph);
    print_line("\tA*");
    experimentsAStar(area_folder, queries, graph);
    print_line("\tBidirectional Landmarks");
    experimentsBidirectionalLandmarks(landmark, landmark_pick_count, area_folder, queries, graph);
    print_line("\tBidirectional A*");
    experimentsBidirectionalAStar(area_folder, queries, graph);
    print_line("\tBidirectional Dijkstra");
    experimentsBidirectionalDijkstra(area_folder, queries, graph);
    print_line("\tDijkstra");
    experimentsDijkstra(area_folder, queries, graph);
}

bool isExistingDirectory(std::string &directory_path) {
    auto file_status = std::filesystem::status(directory_path);
    return std::filesystem::exists(file_status) && std::filesystem::is_directory(file_status);
}

bool constructDirectory(std::string &directory_path) {
    return std::filesystem::create_directories(directory_path);
}

void createAreaFolders(std::vector<std::string> &folders) {
    for (std::string  path : folders) {
        if (!isExistingDirectory(path)) {
            if (constructDirectory(path)) {
                throw std::runtime_error("The output_path " + path + " has been created, but the relevant nodes and edges must be placed there.");
            } else {
                throw std::runtime_error("The output_path " + path + " neither exists nor could be created in the resource directory.");
            }
        }
    }
}

void throwIfNotInFolder(std::string &file, std::string &folder) {
    auto file_status = std::filesystem::status(folder+file);
    if (!std::filesystem::exists(file_status)) {
        throw std::runtime_error("The file "+file+" could not be found at "+folder+file+". Place the file correctly and try again.");
    }
}

int main() {
    int iterations = 100000;
    int landmark_select_count = 16;
    int landmark_pick_count = 3;
    std::vector<std::string> area_folders {(std::string) TEST_RESOURCE_DIR + "/malta"/*,
                                           (std::string) TEST_RESOURCE_DIR + "/bornholm",
                                           (std::string) TEST_RESOURCE_DIR + "/denmark"*/};
    createAreaFolders(area_folders);
    std::string nodes_file = "/nodes.txt";
    std::string edges_file = "/edges.txt";
    std::string reach_bounds_file = "/reach_bounds.txt";
    for (std::string folder : area_folders) {
        throwIfNotInFolder(nodes_file, folder);
        throwIfNotInFolder(edges_file, folder);
        throwIfNotInFolder(reach_bounds_file, folder);
    }

    //for (std::string& area_folder : area_folders) {
    //    runOnAllExperiments(area_folder, iterations, landmark_select_count, landmark_pick_count);
    //}

    landmark_select_count = 64;
    landmark_pick_count = 4;
    for (std::string& area_folder : area_folders) {
        runOnAllExperiments(area_folder, iterations, landmark_select_count, landmark_pick_count);
    }
}