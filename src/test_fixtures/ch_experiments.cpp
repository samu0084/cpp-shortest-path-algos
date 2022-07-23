#include <string>
#include <list>
#include <iostream>
#include <fstream>
#include <queue> 
#include <random>
#include <tuple>
#include "../measurements/snitch.cpp"
#include "../graph/graph.h"
#include "../algorithms/contraction_hierarchies.h"
#include <cmath>
#include <utility>
#include <fstream>

std::vector<std::pair<std::string, std::pair<std::string, std::string>>> paths;
const std::string local_dir = "/home/nkb/Documents/Data";
const std::string OUTPUT_PATH = "/home/nkb/Documents/Outputs/CH";
const int number_queries = 100000;
const auto intervals = {0.1, 0.2, 0.3, 0.4, 0.5, 0.7, 0.8, 0.9, 1.0};

void loadPaths() {
    std::string bornholm_vertices = (local_dir + "/Bornholm/nodes.txt");
    std::string bornholm_edges = (local_dir + "/Bornholm/edges.txt");
    paths.emplace_back("bornholm", std::pair<std::string, std::string>(bornholm_vertices, bornholm_edges));

    std::string newzealand_vertices = (local_dir + "/NewZealand/nodes.txt");
    std::string newzealand_edges = (local_dir + "/NewZealand/edges.txt");
    paths.emplace_back("newzealand", std::pair<std::string, std::string>(newzealand_vertices, newzealand_edges));

    std::string denmark_vertices = (local_dir + "/Denmark/nodes.txt");
    std::string denmark_edges = (local_dir + "/Denmark/edges.txt");
    paths.emplace_back("denmark", std::pair<std::string, std::string>(denmark_vertices, denmark_edges));

    std::string australia_vertices = (local_dir + "/Australia/nodes.txt");
    std::string australia_edges = (local_dir + "/Australia/edges.txt");
    paths.emplace_back("australia", std::pair<std::string, std::string>(australia_vertices, australia_edges));


}

void writeTestData(std::vector<std::tuple<int, int, int, int, long>> data, std::string file) {
    std::ofstream output_file(file, std::ofstream::trunc);
    output_file << "rank,visits,insertions,extractions,microseconds\n";
    for (auto entry : data) {
        output_file << std::get<0>(entry)
        << ',' << std::get<1>(entry)
        << ',' << std::get<2>(entry)
        << ',' << std::get<3>(entry)
        << ',' << std::get<3>(entry)
        << '\n';
    }
    output_file.close();
}

void writeTestConstructData(const std::vector<std::pair<double, long>>& data, const std::string& file) {
    std::ofstream output_file(file, std::ofstream::trunc);
    output_file << "interval,microseconds\n";
    for (auto entry : data) {
        output_file << std::get<0>(entry)
                    << ',' << std::get<1>(entry)
                    << '\n';
    }
    output_file.close();
}

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

std::vector<std::tuple<int, int, int, int, long>> test(ContractionHierarchies& algorithm, Graph& graph, const std::vector<std::pair<int, int>>& queries) {
    snitch snitch;
    std::vector<std::tuple<int, int, int, int, long>> data_points;

    snitch.start_time();
    algorithm.createContractionHierarchy(graph);
    auto preprocessing_microsecs = snitch.stop_time();
    std::cout << "\t\tPreprocessing time : " << std::to_string(preprocessing_microsecs / (1000)) << " miliseconds\n";
    for (auto query : queries) {
        snitch.start_time();
        auto data = algorithm.query(query.first, query.second);
        auto query_microsecs = snitch.stop_time();
        int rank = algorithm.extractPath(std::get<2>(data), std::get<3>(data), std::get<4>(data)).size();
        data_points.emplace_back(rank, std::get<5>(data), std::get<6>(data), std::get<7>(data), query_microsecs);
    }

    return data_points;
}

std::vector<std::pair<double, long>> testConstruct(ContractionHierarchies& algorithm, Graph& graph) {
    snitch snitch;
    std::vector<std::pair<double, long>> data_points;

    for (auto interval : intervals) {
        algorithm.setReconstructionCheckInterval((int) interval * graph.vertices());
        snitch.start_time();
        algorithm.createContractionHierarchy(graph);
        auto preprocessing_microsecs = snitch.stop_time();
        data_points.emplace_back(interval, preprocessing_microsecs);
        std::cout << "\t\tPreprocessing time : " << std::to_string(preprocessing_microsecs / (1000)) << " miliseconds\n";
    }

    return data_points;
}

void compareReconstructionCheckIntervals() {
    std::vector<std::pair<std::string, ContractionHierarchies>> algorithms;
    algorithms.push_back({"EDS10",{true, true, true, 10}}); // EDS5
    algorithms.push_back({"EDS5",{true, true, true, 5}}); // EDS5
    algorithms.push_back({"EDS2",{true, true, true, 5}}); // EDS5
    algorithms.push_back({"ED5",{true, true, false, 5}}); // ED5
    algorithms.push_back({"E5",{true, false, false, 5}}); // E5
    algorithms.push_back({"EDS",{true, true, true, std::numeric_limits<int>::max()}}); // EDS
    algorithms.push_back({"E",{true, false, false, std::numeric_limits<int>::max()}}); // E
    algorithms.push_back({"ES",{true, false, true, std::numeric_limits<int>::max()}}); // ES
    algorithms.push_back({"DS",{false, true, true, std::numeric_limits<int>::max()}}); // DS
    algorithms.push_back({"DS5",{false, true, true, 5}}); // EDS
    algorithms.push_back({"DS10",{false, true, true, 10}}); // EDS

    for (const auto& entry : paths) {

        auto graph_name = entry.first;
        auto paths = entry.second;
        std::ifstream v_stream(paths.first, std::ios::in);
        std::ifstream e_stream(paths.second, std::ios::in);

        std::cout << "Loading " << graph_name << "\n";
        Graph graph = Graph(v_stream, e_stream);
        std::cout << "\t Loaded " << std::to_string(graph.vertices()) << " vertices and " << std::to_string(graph.edges()) << " edges\n";
        v_stream.close();
        e_stream.close();
        for (const auto& name_algorithm : algorithms) {
            auto name = name_algorithm.first;
            std::cout << "\t Testing " << name << "\n";
            auto algorithm = name_algorithm.second;
            auto test_data = testConstruct(algorithm, graph);
            writeTestConstructData(test_data, OUTPUT_PATH + "/"+ graph_name + "_" + name + "_construct.csv");
        }
    }
}

void compareCHs() {
    std::vector<std::pair<std::string, ContractionHierarchies>> algorithms;
    algorithms.push_back({"EDS10",{true, true, true, 10}}); // EDS5
    algorithms.push_back({"EDS5",{true, true, true, 5}}); // EDS5
    algorithms.push_back({"EDS2",{true, true, true, 5}}); // EDS5
    algorithms.push_back({"ED5",{true, true, false, 5}}); // ED5
    algorithms.push_back({"E5",{true, false, false, 5}}); // E5
    algorithms.push_back({"EDS",{true, true, true, std::numeric_limits<int>::max()}}); // EDS
    algorithms.push_back({"E",{true, false, false, std::numeric_limits<int>::max()}}); // E
    algorithms.push_back({"ES",{true, false, true, std::numeric_limits<int>::max()}}); // ES
    algorithms.push_back({"DS",{false, true, true, std::numeric_limits<int>::max()}}); // DS
    algorithms.push_back({"DS5",{false, true, true, 5}}); // EDS
    algorithms.push_back({"DS10",{false, true, true, 10}}); // EDS

    for (auto entry : paths) {

        auto graph_name = entry.first;
        auto paths = entry.second;
        std::ifstream v_stream(paths.first, std::ios::in);
        std::ifstream e_stream(paths.second, std::ios::in);

        std::cout << "Loading " << graph_name << "\n";
        Graph graph = Graph(v_stream, e_stream);
        std::cout << "\t Loaded " << std::to_string(graph.vertices()) << " vertices and " << std::to_string(graph.edges()) << " edges\n";
        v_stream.close();
        e_stream.close();
        std::cout << "Generating " << number_queries << " queries\n";       
        auto queries = generateQueries(number_queries, graph);
        for (auto name_algorithm : algorithms) {
            auto name = name_algorithm.first;
            std::cout << "\t Testing " << name << "\n";
            auto algorithm = name_algorithm.second;
            auto test_data = test(algorithm, graph, queries);
            writeTestData(test_data, OUTPUT_PATH + "/"+ graph_name + "_" + name + ".csv");
        }
    }
}

int main() {
    loadPaths();
    std::cout << "Begun testing ... \n";
    std::cout << "Comparing CH variants ... \n";
    compareCHs();
    std::cout << "Comparing reconstruction intervals ... \n";
    compareReconstructionCheckIntervals();
}