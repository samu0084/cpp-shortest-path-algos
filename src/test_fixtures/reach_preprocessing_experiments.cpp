
#include <string>
#include <iostream>
#include <fstream>
#include <filesystem>
#include "../graph/graph.h"
#include "../algorithms/reach.h"

int infinity = std::numeric_limits<int>::max();


void print_line(std::string msg) {
    std::cout << msg << '\n';
}

void write(std::vector<std::tuple<int, int, long long>> &all_data, std::string &file, bool add_to_end_of_file = false) {
    if (add_to_end_of_file) {
        std::ofstream output_file(file, std::ofstream::app);
        auto entry = all_data[all_data.size()-1];
        auto unbounded_vertices = std::get<0>(entry);
        auto bounded_vertices = std::get<1>(entry);
        auto microsec = std::get<2>(entry);
        output_file << unbounded_vertices
                    << ',' << bounded_vertices
                    << ',' << microsec
                    << '\n';
        output_file.close();
    } else {
        std::ofstream output_file(file, std::ofstream::trunc);
        output_file << "unbounded_vertices,bounded_vertices,microseconds\n";
        for (auto entry : all_data) {
            auto unbounded_vertices = std::get<0>(entry);
            auto bounded_vertices = std::get<1>(entry);
            auto microsec = std::get<2>(entry);
            output_file << unbounded_vertices
                        << ',' << bounded_vertices
                        << ',' << microsec
                        << '\n';
        }
        output_file.close();
    }

}

long long timeStdChronoPreprocessing(Graph &graph, int bound_search_limiter, std::vector<int> &container_for_preprocessing_results) {
    auto start = std::chrono::steady_clock::now();
    Reach::singleReachBoundComputation(graph, bound_search_limiter, container_for_preprocessing_results);
    auto end = std::chrono::steady_clock::now();
    long long time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    return time;
}

std::tuple<int, int> countBoundedUnbounded(std::vector<int> &bounds) {
    int bounded = 0;
    int unbounded = 0;
    for (int bound : bounds) {
        if (bound == infinity) {
            ++unbounded;
        } else {
            ++bounded;
        }
    }
    return {bounded, unbounded};
}

void preprocessingFromRunner(int begin_bound_search_limiter, std::string &vertices_path, std::string &edges_path, std::string &output_path, std::string &file_name, std::vector<int> &bound_search_limiters) {
    std::ifstream v_stream(vertices_path, std::ios::in);
    std::ifstream e_stream(edges_path, std::ios::in);
    print_line("Load graph.");
    Graph graph = Graph(v_stream, e_stream);
    print_line("Perform experiment.");
    std::vector<int> bounds_time_preprocessing;
    std::string previous_reach_file = output_path+"/reach_bounds_"+std::to_string(begin_bound_search_limiter)+".txt";
    std::ifstream b_stream(previous_reach_file, std::ios::in);
    Reach::read(b_stream, bounds_time_preprocessing);
    std::vector<std::tuple<int, int, long long>> data;
    data.reserve(bound_search_limiters.size());
    for (int bound_search_limiter : bound_search_limiters) {
        print_line("Current bound_search_limiter: "+std::to_string(bound_search_limiter));
        // Do experiments
        auto time = timeStdChronoPreprocessing(graph, bound_search_limiter, bounds_time_preprocessing);
        auto [bounded, unbounded] = countBoundedUnbounded(bounds_time_preprocessing);
        data.emplace_back(unbounded, bounded, time);
        // Write reach bounds
        std::string post_fix = std::to_string(bound_search_limiter);
        Reach::write(output_path, post_fix, bounds_time_preprocessing);
        // Write experiment results (The file is overwritten with the newest information)
        std::string file_path = output_path + file_name;
        bool add_to_end_of_file = true;
        write(data, file_path, add_to_end_of_file);
    }
}

void preprocessingRunner(std::string &vertices_path, std::string &edges_path, std::string &output_path, std::string &file_name, std::vector<int> &bound_search_limiters) {
    std::ifstream v_stream(vertices_path, std::ios::in);
    std::ifstream e_stream(edges_path, std::ios::in);
    print_line("Load graph.");
    Graph graph = Graph(v_stream, e_stream);
    print_line("Perform experiment.");
    std::vector<int> bounds_time_preprocessing(graph.vertices(), infinity);
    std::vector<int> bounds_monitor_preprocessing(graph.vertices(), infinity);
    std::vector<std::tuple<int, int, long long>> data;
    data.reserve(bound_search_limiters.size());
    for (int bound_search_limiter : bound_search_limiters) {
        print_line("Current bound_search_limiter: "+std::to_string(bound_search_limiter));
        // Do experiments
        auto time = timeStdChronoPreprocessing(graph, bound_search_limiter, bounds_time_preprocessing);
        auto [bounded, unbounded] = countBoundedUnbounded(bounds_time_preprocessing);
        data.emplace_back(unbounded, bounded, time);
        // Write reach bounds
        std::string post_fix = "_3_"+std::to_string(bound_search_limiter);
        Reach::write(output_path, post_fix, bounds_time_preprocessing);
        // Write experiment results (The file is overwritten with the newest information)
        std::string file_path = output_path + file_name;
        write(data, file_path);
    }
}
//{50, 100, 150, 250, 400, 800, 1000, 1800, 2800, 4600, 7400, 12000, 19400, 31400, 50800}
//{100, 800, 2800, 7400, 19400, 31400, 50800}
//{100, 2800, 19400, 50800}
//{50800}
void preprocessingBornholm(std::string &output_folder_name, std::vector<int> &bound_search_limiters) {
    std::string vertices_path = (std::string) TEST_RESOURCE_DIR + output_folder_name + "/nodes.txt";
    std::string edges_path = (std::string) TEST_RESOURCE_DIR + output_folder_name + "/edges.txt";
    std::string output_path = (std::string) TEST_RESOURCE_DIR + output_folder_name;
    std::string results_file_name = "/preprocessingExperiment5.txt";
    preprocessingRunner(vertices_path, edges_path, output_path, results_file_name, bound_search_limiters);
}

void preprocessingMalta(std::string &output_folder_name, std::vector<int> &bound_search_limiters) {
    std::string vertices_path = (std::string) TEST_RESOURCE_DIR + "/"+output_folder_name+"/nodes.txt";
    std::string edges_path = (std::string) TEST_RESOURCE_DIR + "/"+output_folder_name+"/edges.txt";
    std::string output_path = (std::string) TEST_RESOURCE_DIR + "/"+output_folder_name;
    std::string results_file_name = "/preprocessingExperiment5.txt";
    preprocessingRunner(vertices_path, edges_path, output_path, results_file_name, bound_search_limiters);
}

void preprocessingDenmark(std::string &output_folder_name, std::vector<int> &bound_search_limiters) {
    std::string vertices_path = (std::string) TEST_RESOURCE_DIR + output_folder_name+"/nodes.txt";
    std::string edges_path = (std::string) TEST_RESOURCE_DIR + output_folder_name+"/edges.txt";
    std::string output_path = (std::string) TEST_RESOURCE_DIR + output_folder_name;
    std::string results_file_name = "/preprocessingExperiment5.txt";
    preprocessingRunner(vertices_path, edges_path, output_path, results_file_name, bound_search_limiters);
}

void continuePreprocessingFromMalta(int begin_bound_search_limiter, std::string &output_folder_name, std::vector<int> &bound_search_limiters) {
    std::string vertices_path = (std::string) TEST_RESOURCE_DIR + "/"+output_folder_name+"/nodes.txt";
    std::string edges_path = (std::string) TEST_RESOURCE_DIR + "/"+output_folder_name+"/edges.txt";
    std::string output_path = (std::string) TEST_RESOURCE_DIR + "/"+output_folder_name;
    std::string results_file_name = "/preprocessingExperiment1.txt";
    preprocessingFromRunner(begin_bound_search_limiter, vertices_path, edges_path, output_path, results_file_name, bound_search_limiters);
};

void continuePreprocessingFromDenmark(int begin_bound_search_limiter, std::string &output_folder_name, std::vector<int> &bound_search_limiters) {
    std::string vertices_path = (std::string) TEST_RESOURCE_DIR + output_folder_name+"/nodes.txt";
    std::string edges_path = (std::string) TEST_RESOURCE_DIR + output_folder_name+"/edges.txt";
    std::string output_path = (std::string) TEST_RESOURCE_DIR + output_folder_name;
    std::string results_file_name = "/preprocessingExperiment5.txt";
    preprocessingFromRunner(begin_bound_search_limiter, vertices_path, edges_path, output_path, results_file_name, bound_search_limiters);
}

bool isExistingDirectory(std::string directory_path) {
    auto file_status = std::filesystem::status(directory_path);
    bool exists = std::filesystem::exists(file_status);
    bool is_directory = exists && std::filesystem::is_directory(file_status);
    return exists && is_directory;
}

int main() {
    // NOTE: Folders 'bornholm', 'malta' and 'denmark' should be present in the resource folder beforehand.
    std::string malta_folder = "/malta";
    std::string bornholm_folder = "/bornholm";
    std::string denmark_folder = "/denmark";
    if (!isExistingDirectory(TEST_RESOURCE_DIR+malta_folder)) {
        throw std::runtime_error("The folder 'malta' has not been created in the resource directory.");
    }
    if (!isExistingDirectory(TEST_RESOURCE_DIR+bornholm_folder)) {
        throw std::runtime_error("The folder 'bornholm' has not been created in the resource directory.");
    }
    if (!isExistingDirectory(TEST_RESOURCE_DIR+denmark_folder)) {
        throw std::runtime_error("The folder 'denmark' has not been created in the resource directory.");
    }

    std::vector<int> bound_search_limiters {0};
    print_line("Preprocessing for Malta");
    preprocessingMalta(malta_folder, bound_search_limiters);
    print_line("Preprocessing for Bornholm");
    preprocessingBornholm(bornholm_folder, bound_search_limiters);
    print_line("Preprocessing for Denmark");
    preprocessingDenmark(denmark_folder, bound_search_limiters);
    // The uncommented lines below makes it possible to continue preprocessing
    // from previous bounds, given the corresponding bound search limiter
    //int begin_bound_search_limiter = 50800;
    //continuePreprocessingFromDenmark(begin_bound_search_limiter, denmark_folder, bound_search_limiters);
    print_line("Finished");
}
