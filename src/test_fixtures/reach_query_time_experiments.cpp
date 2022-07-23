#include <string>
#include <iostream>
#include <fstream>
#include <queue>
#include <random>
#include <tuple>
#include "../../modules/post-order-heap/src/post-order_heap.hpp"
#include "../../modules/dary-heap/src/dary_heap.h"
#include "../graph/graph.h"
#include "../algorithms/a_star.h"
#include "../algorithms/a_star_queue_safe.hpp"
#include "../algorithms/reach.h"
#include "../algorithms/a_star_reach.h"
#include "../algorithms/landmark.h"
#include <filesystem>

typedef int Heuristic(int);

int timingRepeats = 1;

enum HeuristicChoice {
    no_heuristic,
    euclidean_distance,
    landmarks
};

void print_line(std::string msg) {
    std::cout << msg << '\n';
}

void write(std::vector<std::tuple<unsigned long long, int, long>> &all_data, std::string &file_path) {
    std::ofstream output_file(file_path, std::ofstream::trunc);
    output_file << "rank,visits,microseconds\n";
    for (auto entry : all_data) {
        auto path_length = std::get<0>(entry);
        auto visits = std::get<1>(entry);
        auto microseconds = std::get<2>(entry);
        output_file << path_length
                    << ',' << visits
                    << ',' << microseconds
                    << '\n';
    }
    output_file.close();
}

std::tuple<unsigned long long, int, long long> timeSingleQuery(int origin, int destination, Graph &graph, AStar &a_star) {
    Data query_results;
    long long totalTime = 0;
    for (int i = 0; i < timingRepeats; ++i) {
        auto start_time = std::chrono::steady_clock::now();
        query_results = a_star.run(graph, origin, destination);
        totalTime += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start_time).count();
    }
    int visits = std::get<2>(query_results);
    unsigned long long rank = AStarReach::extractPath(std::get<1>(query_results), destination).size();
    return {rank, visits, totalTime/timingRepeats};
}

std::tuple<unsigned long long, int, long long> timeSingleQuery(Landmark &landmark, int origin, int destination, Graph &graph, AStar &a_star) {
    Data query_results;
    long long totalTime = 0;
    for (int i = 0; i < timingRepeats; ++i) {
        int recommended_number_of_active_landmarks = 3;
        auto start_time = std::chrono::steady_clock::now();
        std::vector<int> picked_landmarks = landmark.pick(graph.getCoordinates(origin),
                                                          graph.getCoordinates(destination),
                                                          recommended_number_of_active_landmarks);
        landmark.activate(picked_landmarks);
        landmark.setDestination(destination);
        query_results = a_star.run(graph, origin, destination);
        totalTime += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start_time).count();
    }
    int visits = std::get<2>(query_results);
    unsigned long long rank = AStarReach::extractPath(std::get<1>(query_results), destination).size();
    return {rank, visits, totalTime/timingRepeats};
}

std::tuple<unsigned long long, int, long long> timeSingleQueryWithExternal(Landmark &landmark, int origin, int destination, Graph &graph, AStarReach &a_star_reach) {
    Data query_results;
    long long totalTime = 0;
    for (int i = 0; i < timingRepeats; ++i) {
        int recommended_number_of_active_landmarks = 3;
        auto start_time = std::chrono::steady_clock::now();
        std::vector<int> picked_landmarks = landmark.pick(graph.getCoordinates(origin),
                                                          graph.getCoordinates(destination),
                                                          recommended_number_of_active_landmarks);
        landmark.activate(picked_landmarks);
        landmark.setDestination(destination);
        query_results = a_star_reach.run(graph, origin, destination);
        totalTime += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start_time).count();
        landmark.deactivate();
    }
    int visits = std::get<2>(query_results);
    unsigned long long rank = AStarReach::extractPath(std::get<1>(query_results), destination).size();
    return {rank, visits, totalTime/timingRepeats};
}

std::tuple<unsigned long long, int, long long> timeSingleQueryWithoutExternal(Landmark &landmark, int origin, int destination, Graph &graph, AStarReach &a_star_reach) {
    Data query_results;
    long long totalTime = 0;
    for (int i = 0; i < timingRepeats; ++i) {
        int recommended_number_of_active_landmarks = 3;
        auto start_time = std::chrono::steady_clock::now();
        std::vector<int> picked_landmarks = landmark.pick(graph.getCoordinates(origin),
                                                          graph.getCoordinates(destination),
                                                          recommended_number_of_active_landmarks);
        landmark.activate(picked_landmarks);
        landmark.setDestination(destination);
        query_results = a_star_reach.runWithoutExternalHeuristic(graph, origin, destination);
        totalTime += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start_time).count();
        landmark.deactivate();
    }
    int visits = std::get<2>(query_results);
    unsigned long long rank = AStarReach::extractPath(std::get<1>(query_results), destination).size();
    return {rank, visits, totalTime/timingRepeats};
}

std::tuple<unsigned long long, int, long long> timeSingleQueryWithExternal(int origin, int destination, Graph &graph, AStarReach &a_star_reach) {
    Data query_results;
    long long totalTime = 0;
    for (int i = 0; i < timingRepeats; ++i) {
        auto start_time = std::chrono::steady_clock::now();
        query_results = a_star_reach.run(graph, origin, destination);
        totalTime += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start_time).count();
    }
    int visits = std::get<2>(query_results);
    unsigned long long rank = AStarReach::extractPath(std::get<1>(query_results), destination).size();
    return {rank, visits, totalTime/timingRepeats};
}



std::tuple<long long, int, long long> timeSingleQueryWithoutExternal(int origin, int destination, Graph &graph, AStarReach &a_star_reach) {
    Data query_results;
    long long totalTime = 0;
    for (int i = 0; i < timingRepeats; ++i) {
        auto start_time = std::chrono::steady_clock::now();
        query_results = a_star_reach.runWithoutExternalHeuristic(graph, origin, destination);
        totalTime += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start_time).count();
    }
    int visits = std::get<2>(query_results);
    unsigned long long rank = AStarReach::extractPath(std::get<1>(query_results), destination).size();
    return {rank, visits, totalTime/timingRepeats};
}

void timeAStarTestRunner(HeuristicChoice heuristic_choice, int iterations, Graph &graph, std::string &output_folder_path) {
    Landmark landmark(graph);
    if (heuristic_choice == HeuristicChoice::landmarks) {
        int recommended_number_of_landmarks = 16;
        std::cout << "  Selecting landmarks";
        std::vector<int> all_landmarks = landmark.selectFarthestByBFS(recommended_number_of_landmarks);
        std::cout << " - done" << std::endl << "  Generating landmarks";
        landmark.generate(all_landmarks);
        std::cout << " - done" << std::endl;
    }

    std::mt19937 gen(0); // The none-randomness of the seed is intentional. The experiments should be reproducible.
    std::uniform_int_distribution<> uniformIntDistributionRange(0, graph.vertices()-1);

    std::vector<std::tuple<unsigned long long, int, long>> results;
    results.reserve(iterations);
    std::cout   << "  timeAStarTestRunner("
                << heuristic_choice << ", "
                << iterations << " iterations, " << std::endl
                << "                      "
                << output_folder_path << std::endl;
    for (int iteration = 0; iteration < iterations; ++iteration) {
        int origin = uniformIntDistributionRange(gen);
        int destination = uniformIntDistributionRange(gen);

        std::function<Heuristic> heuristic;
        switch (heuristic_choice) {
            case no_heuristic: heuristic = [](int id) { return 0; }; break;
            case euclidean_distance: heuristic = [&graph, &destination](int id) {
                    return (int) computeWeight(
                            graph.getCoordinates(id),graph.getCoordinates(destination));
                }; break;
            case landmarks: heuristic = [&landmark](int id) {
                    return landmark.apply(id);
                }; break;
        }

        AStar a_star(heuristic);

        std::tuple<int, int, long> result;
        if (heuristic_choice == HeuristicChoice::landmarks) {
            result = timeSingleQuery(landmark, origin, destination, graph, a_star);
        } else if (heuristic_choice == HeuristicChoice::euclidean_distance || heuristic_choice == HeuristicChoice::no_heuristic) {
            result = timeSingleQuery(origin, destination, graph, a_star);
        } else {
            throw std::runtime_error("The heuristic choice must be landmarks or euclidean distance");
        }
        results.emplace_back(result);
        if ((iteration+1) % 10000 == 0) {
            auto time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            std::cout   << std::endl
                        << std::put_time(std::localtime(&time), "%Y-%m-%d %X")
                        << "    Performed " << iteration+1 << " iterations"
                        << std::endl;
        }
    }
    auto time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::cout   << std::endl
                << std::put_time(std::localtime(&time), "%Y-%m-%d %X")
                << "    Done after " << iterations << " iterations"
                << std::endl;
    // Write to file (keep the bounds precision and use of external heuristic and which heuristic in the title)
    std::string heuristic_choice_string;
    if (heuristic_choice == HeuristicChoice::landmarks)
        heuristic_choice_string = "landmarks";
    else if (heuristic_choice == HeuristicChoice::euclidean_distance)
        heuristic_choice_string = "a_star";
    else // heuristic_choice == HeuristicChoice::no_heuristic
        heuristic_choice_string = "dijkstra";
    std::string output_file_path = output_folder_path + "/query_experiments_using_bounds_" + heuristic_choice_string + ".txt";
    write(results, output_file_path);
}

void timeTestRunner(HeuristicChoice heuristic_choice, bool use_external_heuristic, int iterations, Graph &graph, Reach &reach,
                    int bounds_precision, std::string &output_folder_path) {
    Landmark landmark(graph);
    if (heuristic_choice == HeuristicChoice::landmarks) {
        int recommended_number_of_landmarks = 16;
        std::cout << "  Selecting landmarks";
        std::vector<int> all_landmarks = landmark.selectFarthestByBFS(recommended_number_of_landmarks);
        std::cout << " - done" << std::endl << "  Generating landmarks";
        landmark.generate(all_landmarks);
        std::cout << " - done" << std::endl;
        std::cout << "  Landmarks selected: ";
        for (int landmark_id : all_landmarks) {
            std::cout << " " << landmark_id;
        }
        std::cout << std::endl;
    }

    std::mt19937 gen(0); // The none-randomness of the seed is intentional. The experiments should be reproducible.
    std::uniform_int_distribution<> uniformIntDistributionRange(0, graph.vertices()-1);

    std::vector<std::tuple<unsigned long long, int, long>> results;
    results.reserve(iterations);
    std::cout   << "  timeTestRunner("
                << heuristic_choice << ", "
                << (use_external_heuristic ? "use heuristic, " : "do not use heuristic, ")
                << iterations << " iterations, " << std::endl
                << "                 "
                << bounds_precision << " bounds_search_limiter, "
                << output_folder_path << std::endl;
    for (int iteration = 0; iteration < iterations; ++iteration) {
        int origin = uniformIntDistributionRange(gen);
        int destination = uniformIntDistributionRange(gen);

        std::function<Heuristic> heuristic;
        switch (heuristic_choice) {
            case no_heuristic: heuristic = [](int id) { return 0; }; break;
            case euclidean_distance:
                heuristic = [&graph, &destination](int id) {
                    return (int) computeWeight(
                            graph.getCoordinates(id),graph.getCoordinates(destination));
                }; break;
            case landmarks:
                heuristic = [&landmark](int id) {
                return landmark.apply(id);
                }; break;
        }

        AStarReach a_star_reach(heuristic, reach);

        std::tuple<unsigned long long, int, long> result;
        if (heuristic_choice == HeuristicChoice::landmarks && use_external_heuristic) {
            result = timeSingleQueryWithExternal(landmark, origin, destination, graph, a_star_reach);
        } else if (heuristic_choice == HeuristicChoice::landmarks && !use_external_heuristic) {
            result = timeSingleQueryWithoutExternal(landmark, origin, destination, graph, a_star_reach);
        } else if (heuristic_choice == HeuristicChoice::euclidean_distance && use_external_heuristic) {
            result = timeSingleQueryWithExternal(origin, destination, graph, a_star_reach);
        } else if (heuristic_choice == HeuristicChoice::euclidean_distance && !use_external_heuristic) {
            result = timeSingleQueryWithoutExternal(origin, destination, graph, a_star_reach);
        } else {
            throw std::runtime_error("The heuristic choice must be landmarks or euclidean distance");
        }
        results.emplace_back(result);
        if ((iteration+1) % 1000 == 0) {
            auto time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            std::cout   << std::endl
                        << std::put_time(std::localtime(&time), "%Y-%m-%d %X")
                        << "    Performed " << iteration+1 << " iterations"
                        << std::endl;
        }
    }
    auto time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::cout   << std::endl
                << std::put_time(std::localtime(&time), "%Y-%m-%d %X")
                << "    Done after " << iterations << " iterations"
                << std::endl;
    // Write to file (keep the bounds precision and use of external heuristic and which heuristic in the title)
    std::string heuristic_choice_string;
    if (heuristic_choice == HeuristicChoice::landmarks)
        heuristic_choice_string = "_landmarks";
    else
        heuristic_choice_string = "_euclideanDistance";
    std::string use_external_heuristic_string;
    if (use_external_heuristic)
        use_external_heuristic_string = "_withExternalHeuristic";
    else
        use_external_heuristic_string = "_withoutExternalHeuristic";
    std::string bounds_precision_string = std::to_string(bounds_precision);
    std::string output_file_path = output_folder_path + "/2query_experiments_using_bounds_" + bounds_precision_string + use_external_heuristic_string + heuristic_choice_string + ".txt";
    write(results, output_file_path);
}

/**
 *
 * @param iterations
 * @param area The name of the area under test in small letters. This name should correspond to a folder name inside of
 * resources where the nodes, edges and bounds of the specified precision is saved.
 * @param bounds_precision
 */
void testReachBased(int iterations, std::string &area_folder, std::vector<int> &bounds_precisions) {
    std::string output_folder_path = (std::string) area_folder;

    std::cout << "  Loading graph";
    std::string vertices_path = (std::string) area_folder + "/nodes.txt";
    std::string edges_path = (std::string) area_folder + "/edges.txt";
    std::ifstream v_stream(vertices_path, std::ios::in);
    std::ifstream e_stream(edges_path, std::ios::in);
    Graph graph(v_stream, e_stream);
    v_stream.close();
    e_stream.close();
    std::cout << " - done" << std::endl;
    // Do all experiments for landmarks
    for (int bounds_precision : bounds_precisions) {
        std::cout << "  Loading reach-bounds";
        std::string reach_bounds_path = (std::string) area_folder + "/reach_bounds_"+std::to_string(bounds_precision)+".txt";
        std::ifstream reach_stream(reach_bounds_path, std::ios::in);
        std::vector<int> reach_bounds;
        Reach::read(reach_stream, reach_bounds);
        reach_stream.close();
        Reach reach(reach_bounds);
        std::cout << " - done" << std::endl;

        timeTestRunner(landmarks, true, iterations, graph, reach, bounds_precision, output_folder_path);
        timeTestRunner(landmarks, false, iterations, graph, reach, bounds_precision, output_folder_path);
    }
    // Do all experiments for euclidean distance
    for (int bounds_precision : bounds_precisions) {
        std::cout << "  Loading reach-bounds";
        std::string reach_bounds_path = (std::string) area_folder + "/reach_bounds_"+std::to_string(bounds_precision)+".txt";
        std::ifstream reach_stream(reach_bounds_path, std::ios::in);
        std::vector<int> reach_bounds;
        Reach::read(reach_stream, reach_bounds);
        reach_stream.close();
        Reach reach(reach_bounds);
        std::cout << " - done" << std::endl;

        timeTestRunner(euclidean_distance, true, iterations, graph, reach, bounds_precision, output_folder_path);
        timeTestRunner(euclidean_distance, false, iterations, graph, reach, bounds_precision, output_folder_path);
    }
}

void testAStar(int iterations, std::string &area_folder) {
    std::string output_folder_path = (std::string) area_folder;

    std::cout << "  Loading graph";
    std::string vertices_path = (std::string) area_folder + "/nodes.txt";
    std::string edges_path = (std::string) area_folder + "/edges.txt";
    std::ifstream v_stream(vertices_path, std::ios::in);
    std::ifstream e_stream(edges_path, std::ios::in);
    Graph graph(v_stream, e_stream);
    v_stream.close();
    e_stream.close();
    std::cout << " - done" << std::endl;
    int bounds_precision = 0;
    // Do all experiments for landmarks
    timeAStarTestRunner(landmarks, iterations, graph, output_folder_path);
    // Do all experiments for euclidean distance
    timeAStarTestRunner(euclidean_distance, iterations, graph, output_folder_path);
    // Do all experiments for pure dijkstra
    timeAStarTestRunner(no_heuristic, iterations, graph, output_folder_path);
}

bool isExistingDirectory(std::string &directory_path) {
    auto file_status = std::filesystem::status(directory_path);
    return std::filesystem::exists(file_status) && std::filesystem::is_directory(file_status);
}

int main() {
    int iterations = 20000;
    // NOTE: Folders 'bornholm', 'malta' and 'denmark' should contain relevant reaches computations beforehand.
    // Check that folders exists
    std::string malta_folder = (std::string) TEST_RESOURCE_DIR+"/malta";
    std::string bornholm_folder = (std::string) TEST_RESOURCE_DIR+"/bornholm";
    std::string denmark_folder = (std::string) TEST_RESOURCE_DIR+"/denmark";
    if (!isExistingDirectory(malta_folder)) {
        throw std::runtime_error("The folder 'malta' has not been created in the resource directory.");
    }
    if (!isExistingDirectory(bornholm_folder)) {
        throw std::runtime_error("The folder 'bornholm' has not been created in the resource directory.");
    }
    if (!isExistingDirectory(denmark_folder)) {
        throw std::runtime_error("The folder 'denmark' has not been created in the resource directory.");
    }


    print_line("Testing on Bornholm");
    std::vector<int> bounds_precisions_bornholm {0, 50, 100, 150, 250, 400, 800, 1000, 1800, 2800, 4600, 7400, 12000};
    testReachBased(iterations, bornholm_folder, bounds_precisions_bornholm);
    testAStar(iterations, bornholm_folder);

    print_line("Testing on Malta");
    std::vector<int> bounds_precisions_malta {0, 50, 100, 150, 250, 400, 800, 1000, 1800, 2800, 4600, 7400, 12000};
    testReachBased(iterations, malta_folder, bounds_precisions_malta);
    testAStar(iterations, malta_folder);

    print_line("Testing on Denmark");
    std::vector<int> bounds_precisions_denmark{0, 50, 100, 150, 250, 400, 800, 1000, 1800, 2800, 4600, 7400, 12000, 19400, 31400, 50800};
    testReachBased(iterations, denmark_folder, bounds_precisions_denmark);
    testAStar(iterations, denmark_folder);


    print_line("Finished");
}
