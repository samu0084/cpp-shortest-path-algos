#include <string>
#include <list>
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
#include "../algorithms/landmark.h"
#include "../measurements/snitch.h"
#include <cmath>
#include <fstream>

int timingRepeats = 1;

typedef std::vector<int> TestMethod();
typedef Data QueryTest(int origin, int destination);

std::vector<std::pair<int, int>> generateQueries(int n, Graph& graph) {
    int lower = 0;
    int upper = graph.vertices() - 1;
    typedef std::mt19937 range_type;
    std::uniform_int_distribution<range_type::result_type> uniform_distribution(lower, upper);
    range_type range;
    range_type::result_type const seedval = time(nullptr); // get this from somewhere
    range.seed(seedval);
    std::vector<std::pair<int, int>> output;
    for (int index = 0; index < n; index++) {
        output.emplace_back(uniform_distribution(range), uniform_distribution(range));
    }
    return output;
}

std::tuple<long, Data> timePathQuery(int origin, int destination, const std::function<QueryTest> &queryTest) {
    snitch snitch;
    long average_time = 0;
    Data result;
    for (int iteration = 0; iteration < timingRepeats; iteration++) {
        snitch.start_time();
        result = queryTest(origin, destination);
        average_time += snitch.stop_time();
    }
    return {average_time / timingRepeats, result};
}

process_data monitorPathQuery(int origin, int destination, const std::function<QueryTest> &queryTest) {
    snitch snitch;
    snitch.start();
    queryTest(origin, destination);
    snitch.stop();
    return snitch.read_data();
}

std::vector<std::tuple<int, process_data, long, int>>
timeMonitorPathQueryIteratively(std::vector<std::pair<int, int>> queries, const std::function<QueryTest> &queryTest) {

    std::vector<std::tuple<int, process_data, long, int>> results;
    results.reserve(queries.size());

    snitch snitch;
    for (int iteration = 0; iteration < queries.size(); ++iteration) {
        if (iteration % 1000 == 0) {
            auto current_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            std::cout << std::ctime(&current_time) << "    iteration: " << iteration << std::endl;
        }
        auto query = queries[iteration];
        int origin =  query.first;
        int destination = query.second;

        std::tuple<int, Data> result = timePathQuery(origin, destination, queryTest);
        auto s = std::get<1>(result);
        process_data data = monitorPathQuery(origin, destination, queryTest);

        results.emplace_back(
                AStar::extractPath(std::get<1>(s),destination).size(),
                data,  //processed data
                get<0>(result), // duration
                get<2>(get<1>(result))// visit
        );
    }
    return results;
}

std::vector<std::tuple<int, process_data, long, int>> timeMonitorMethod(int iterations, const std::function<TestMethod> &testMethod) {
    std::vector<std::tuple<int, process_data, long, int>> results;
    results.reserve(iterations);
    for (int iteration = 0; iteration < iterations; ++iteration) {
        if (iteration % 1000 == 0) {
            auto current_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            std::cout << std::ctime(&current_time) << "    iteration: " << iteration << std::endl;
        }
        snitch snitch;
        // Measure execution time
        long time = 0;
        for (int i = 0; i < timingRepeats; ++i) {
            snitch.start_time();
            testMethod();
            time += snitch.stop_time();
        }
        // Measure other stuff
        snitch.start();
        testMethod();
        snitch.stop();
        // Results are: Duration
        results.emplace_back(0,snitch.read_data(), time/timingRepeats, 0);
    }
    return results;
}

void print_line(std::string msg) {
    std::cout << msg << '\n';
}

void write(std::vector<std::tuple<int, process_data,long, int>> data, std::string file) {
    std::ofstream output_file(file, std::ofstream::trunc);
    output_file << "n,instructions,page faults,cache references,cache misses,branches,branch misses,visits,microseconds, \n";
    for (auto entry : data) {
        auto n = std::get<0>(entry);
        auto data = std::get<1>(entry);
        auto microsec = std::get<2>(entry);
        auto visits = std::get<3>(entry);
        output_file << n
                    << ',' << std::get<0>(data)
                    << ',' << std::get<1>(data)
                    << ',' << std::get<2>(data)
                    << ',' << std::get<3>(data)
                    << ',' << std::get<4>(data)
                    << ',' << std::get<5>(data)
                    << ',' << visits
                    << ',' << microsec
                    << '\n';
    }
    output_file.close();
}

void timeIncreasingSelectedAndPicked(std::string &output_folder_path, Graph &graph, Landmark &landmark) {
    int max_select = 65;
    int max_pick = 6;
    int iterations = 100000;

    auto queries = generateQueries(iterations,graph);
    for (int number_of_selected = 1; number_of_selected < max_select; number_of_selected *= 2) {
        auto  selected_ids =  landmark.selectFarthestByEuclideanDistance(number_of_selected);
        landmark.generate(selected_ids);
        for (int number_of_picked = 1; number_of_picked <= number_of_selected && number_of_picked <= max_pick; ++number_of_picked) {
            static auto landmarkHeuristic = [&landmark](int id) {
                return (int) landmark.apply(id);
            };
            AStar aStar(landmarkHeuristic);
            static auto test = [&graph, &landmark, &aStar, &number_of_picked](int origin, int destination) {
                // std::cout << "number of picked " << number_of_picked << std::endl;
                auto picked_landmarks = landmark.pick(graph.getCoordinates(origin), graph.getCoordinates(destination), number_of_picked);
                landmark.activate(picked_landmarks);
                landmark.setDestination(destination);
                return aStar.run(graph, origin, destination);
            };


            std::vector<std::tuple<int, process_data, long, int>> results = timeMonitorPathQueryIteratively(queries, test);

            write(results, output_folder_path+"/Selected"+std::to_string(number_of_selected)+"Picked"+std::to_string(number_of_picked)+".txt");
        }
    }
}

void testPreprocessing(std::string &output_folder_path, Landmark &landmark, int vertices) {
    int iterations = 100;
    int number_of_landmarks = 16;

    std::vector<std::tuple<int, process_data, long, int>> results = timeMonitorMethod(iterations, [&landmark, number_of_landmarks]() {
        return landmark.selectFarthestByBFS(number_of_landmarks);
    });

    write(results, output_folder_path+"/select"+std::to_string(number_of_landmarks)+"FarthestByBFS.txt");

    results = timeMonitorMethod(iterations, [&landmark, number_of_landmarks]() {
        return landmark.selectFarthestByShortestPaths(number_of_landmarks);
    });

    write(results, output_folder_path+"/select"+std::to_string(number_of_landmarks)+"FarthestByShortestPaths.txt");

    results = timeMonitorMethod(iterations, [&landmark, number_of_landmarks]() {
        return landmark.selectFarthestByEuclideanDistance(number_of_landmarks);
    });

    write(results, output_folder_path+"/select"+std::to_string(number_of_landmarks)+"FarthestByEuclideanDistance.txt");
}

int main() {
    print_line("Running experiments for landmarks");
    //std::vector<std::string> graphs{"malta","bornholm","denmark"};
    std::string output_folder_path = (std::string) TEST_RESOURCE_DIR;
    print_line("Loading graph");
    std::ifstream v_stream(output_folder_path+"/nodes.txt", std::ios::in);
    std::ifstream e_stream(output_folder_path+"/edges.txt", std::ios::in);
    Graph graph(v_stream, e_stream);
    Landmark landmark(graph);

    print_line("Test preprocessing time of landmark selection variants");
    int vertices = graph.vertices();
    testPreprocessing(output_folder_path, landmark, vertices);

    //print_line("Test whether increasing the SELECTED and PICKED number of landmarks increase the query time");
    //timeIncreasingSelectedAndPicked(output_folder_path, graph, landmark);

    /*
    void testPreprocessing(std::string &output_folder_path, Landmark &landmark, int vertices) {
    int iterations = 1000;
    int number_of_landmarks = 16;
    std::cout << "testPreprocessing: selectFarthestByBFS" << std::endl;
    std::vector<std::tuple<int, process_data, long, int>> results = timeMonitorMethod(iterations, [&landmark, number_of_landmarks]() {
        return landmark.selectFarthestByBFS(number_of_landmarks);
    });
    for (auto & result : results) {
        std::get<0>(result) = vertices;
    }
    write(results, output_folder_path+"/select"+std::to_string(number_of_landmarks)+"FarthestByBFS.txt");

    std::cout << "testPreprocessing: selectFarthestByShortestPaths" << std::endl;
    results = timeMonitorMethod(iterations, [&landmark, number_of_landmarks]() {
        return landmark.selectFarthestByShortestPaths(number_of_landmarks);
    });
    for (auto & result : results) {
        std::get<0>(result) = vertices;
    }
    write(results, output_folder_path+"/select"+std::to_string(number_of_landmarks)+"FarthestByShortestPaths.txt");

    std::cout << "testPreprocessing: selectFarthestByEuclideanDistance" << std::endl;
    results = timeMonitorMethod(iterations, [&landmark, number_of_landmarks]() {
        return landmark.selectFarthestByEuclideanDistance(number_of_landmarks);
    });
    for (auto & result : results) {
        std::get<0>(result) = vertices;
    }
    write(results, output_folder_path+"/select"+std::to_string(number_of_landmarks)+"FarthestByEuclideanDistance.txt");
}


    }
     */
}

enum LandmarkSelectionVariant {
    FurthestBFS,
    FurthestShortestPath,
    FurthestEuclideanDistance
};