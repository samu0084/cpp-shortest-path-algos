#include <string>
#include <list>
#include <iostream>
#include <fstream>
#include <queue> 
#include <random>
#include <tuple>
#include "../measurements/snitch.cpp"
#include "../../modules/post-order-heap/src/post-order_heap.hpp"
#include "../../modules/dary-heap/src/dary_heap.h"
#include "../graph/graph.h"
#include "../algorithms/a_star.h"
#include "../algorithms/a_star_queue_safe.hpp"
#include <cmath>
#include <fstream>



const int exponent_limit = 48;
const int repeats = 5;
const int queries = 100000;
const int exp_start = 20;
const float c = 0.5;
const int optimal_postorder_degree = 3;
const int optimal_dary_degree = 3;
const std::string output_directory = "/home/nkb/Documents/Outputs/Q";

void print_line(std::string msg) {
    std::cout << msg << '\n';
}

auto less = [](int left, int right) {
    return left < right;
};
auto greater = [](int left, int right) {
    return left > right;
};

template<int degree> using postorder =  postorder_heap<degree, int>;
template<int degree> using dary =  dary_heap<degree, int>;
typedef binary_heap_topdown<int> bin_heap;
typedef std::priority_queue<int, std::vector<int>, std::greater<int>> std_queue;

template<typename Queue>
long timeSort(std::vector<int>& integers) {
    snitch snitch;
    int average_time = 0;
    std::vector<int> sorted;
    for (int iter = 0; iter < repeats; iter++) {
        Queue queue;
        snitch.start_time();
        for (auto integer : integers)
            queue.push(integer);
        while (!queue.empty()) {
            int top = queue.top();
            queue.pop();
            sorted.push_back(top);
        }
        average_time += snitch.stop_time();
    }
    return average_time / repeats;
}

template<typename Queue>
process_data monitorSort(std::vector<int>& integers) {
    snitch snitch;
    Queue queue;
    snitch.start();
    std::vector<int> sorted;
    for (auto integer : integers)
        queue.push(integer);
    while (!queue.empty()) {
        int top = queue.top();
        queue.pop();
        sorted.push_back(top);
    }
    snitch.stop();
    return snitch.read_data();
}

long timePostorderDijkstra(Graph graph, int source, int target) {
    snitch snitch;
    AStar dijkstra([](int id) { return 0; });
    int average_time = 0;
    for (int iter = 0; iter < repeats; iter++) {
        snitch.start_time();
        dijkstra.run(graph, source, target);
        average_time += snitch.stop_time();
    }
    return average_time / repeats;
}


process_data monitorPostorderDijkstra(Graph graph, int source, int target) {
    snitch snitch;
    AStar dijkstra([](int id) { return 0; });
    snitch.start();
    dijkstra.run(graph, source, target);
    snitch.stop();
    return snitch.read_data();
}


long timeBinaryDijkstra(Graph graph, int source, int target) {
    snitch snitch;
    AStar dijkstra([](int id) { return 0; });
    int average_time = 0;
    for (int iter = 0; iter < repeats; iter++) {
        snitch.start_time();
        dijkstra.run_binary(graph, source, target);
        average_time += snitch.stop_time();
    }
    return average_time / repeats;
}

process_data monitorBinaryDijkstra(Graph graph, int source, int target) {
    snitch snitch;
    AStar dijkstra([](int id) { return 0; });
    snitch.start();
    dijkstra.run_binary(graph, source, target);
    snitch.stop();
    return snitch.read_data();
}

long timeStdDijkstra(Graph graph, int source, int target) {
    snitch snitch;
    AStarQ<std::priority_queue<std::pair<int,int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>>> dijkstra([](int id) { return 0; });
    int average_time = 0;
    for (int iter = 0; iter < repeats; iter++) {
        snitch.start_time();
        dijkstra.run(graph, source, target);
        average_time += snitch.stop_time();
    }
    return average_time / repeats;
}

process_data monitorStdDijkstra(Graph graph, int source, int target) {
    snitch snitch;
    AStarQ<std::priority_queue<std::pair<int,int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>>> dijkstra([](int id) { return 0; });
    snitch.start();
    dijkstra.run(graph, source, target);
    snitch.stop();
    return snitch.read_data();
}

int determineRank(Graph graph, int source, int target) {
    AStarQ<postorder_heap<optimal_postorder_degree, std::pair<int,int>>> dijkstra([](int id) { return 0; });
    auto data = dijkstra.run(graph, source, target);
    auto path = dijkstra.extractPath(std::get<1>(data), target);
    return path.size();
}

std::vector<int> generateRandom(int n, int lower, int upper) {
    typedef std::mt19937 range_type;
    std::uniform_int_distribution<range_type::result_type> uniform_distribution(lower, upper);
    range_type range;
    range_type::result_type const seedval = time(nullptr); // get this from somewhere
    range.seed(seedval);
    std::vector<int> output;
    for (int index = 0; index < n; index++)
        output.push_back(uniform_distribution(range));
    return output;
}

std::vector<int> generateRandom(int n) {
    return generateRandom(n, std::numeric_limits<int>::min(), std::numeric_limits<int>::max());
}

void write(std::vector<std::tuple<int, process_data,long>> data, std::string file) {
    std::ofstream output_file(file, std::ofstream::trunc);
    output_file << "n,instructions,page faults,cache references,cache misses,branches,branch misses,microseconds\n";
    for (auto entry : data) {
        auto n = std::get<0>(entry);
        auto data = std::get<1>(entry);
        auto microsec = std::get<2>(entry);
        output_file << n
        << ',' << std::get<0>(data)
        << ',' << std::get<1>(data)
        << ',' << std::get<2>(data)
        << ',' << std::get<3>(data)
        << ',' << std::get<4>(data)
        << ',' << std::get<5>(data)
        << ',' << microsec
        << '\n';
    }
    output_file.close();
}

process_data sum(const process_data& a, const process_data& b) {
    return {std::get<0>(a)+std::get<0>(b), std::get<1>(a)+std::get<1>(b), std::get<2>(a)+std::get<2>(b), std::get<3>(a)+std::get<3>(b), std::get<4>(a)+std::get<4>(b), std::get<5>(a)+std::get<5>(b)};
};

process_data divide(const process_data& a, int div) {
    return {std::get<0>(a)/div, std::get<1>(a)/div, std::get<2>(a)/div, std::get<3>(a)/div, std::get<4>(a)/div, std::get<5>(a)/div};
};

void testPostorderSorted(bool comparator(int, int), std::string suffix) {
    std::vector<std::tuple<int, process_data,long>> two;
    std::vector<std::tuple<int, process_data,long>> three;
    std::vector<std::tuple<int, process_data,long>> four;
    std::vector<std::tuple<int, process_data,long>> five;
    std::vector<std::tuple<int, process_data,long>> standard;
    std::vector<std::tuple<int, process_data,long>> topdown;
    
    for (int exponent = exp_start; exponent < exponent_limit;  exponent++) {
        int n = exp2(c*exponent);
        auto integers = generateRandom(n);
        std::sort(integers.begin(), integers.end(), comparator);
        print_line("\t Testing "+std::to_string(n) + " pushes / pops...");

        auto standard_time = timeSort<std::priority_queue<int>>(integers);
        auto standard_data = monitorSort<std::priority_queue<int>>(integers);
        standard.emplace_back(n, standard_data, standard_time);

        auto topdown_time = timeSort<bin_heap>(integers);
        auto topdown_data = monitorSort<bin_heap>(integers);
        topdown.emplace_back(n, topdown_data, topdown_time);

        auto three_time = timeSort<postorder<3>>(integers);
        auto three_data = monitorSort<postorder<3>>(integers);
        three.emplace_back(n, three_data, three_time);

        auto two_time = timeSort<postorder<2>>(integers);
        auto two_data = monitorSort<postorder<2>>(integers);
        two.emplace_back(n, two_data, two_time);

        auto four_time = timeSort<postorder<4>>(integers);
        auto four_data = monitorSort<postorder<4>>(integers);
        four.emplace_back(n, four_data, four_time);

        auto five_time = timeSort<postorder<5>>(integers);
        auto five_data = monitorSort<postorder<5>>(integers);
        five.emplace_back(n, five_data, five_time);
    }
    write(two, output_directory+"/postorder_two"+suffix);
    write(three, output_directory+"/postorder_three"+suffix);
    write(four, output_directory+"/postorder_four"+suffix);
    write(five, output_directory+"/postorder_five"+suffix);
    write(standard, output_directory+"/postorder_standard"+suffix);
    write(topdown, output_directory+"/postorder_topdown"+suffix);
}

void testPostorder() {
    std::vector<std::tuple<int, process_data,long>> two;
    std::vector<std::tuple<int, process_data,long>> three;
    std::vector<std::tuple<int, process_data,long>> four;
    std::vector<std::tuple<int, process_data,long>> five;
    std::vector<std::tuple<int, process_data,long>> standard;
    std::vector<std::tuple<int, process_data,long>> topdown;
    
    for (int exponent = exp_start; exponent < exponent_limit;  exponent++) {
        int n = exp2(c*exponent);
        auto integers = generateRandom(n);
        print_line("\t Testing "+std::to_string(n) + " pushes / pops...");

        auto standard_time = timeSort<std::priority_queue<int>>(integers);
        auto standard_data = monitorSort<std::priority_queue<int>>(integers);
        standard.emplace_back(n, standard_data, standard_time);

        auto topdown_time = timeSort<bin_heap>(integers);
        auto topdown_data = monitorSort<bin_heap>(integers);
        topdown.emplace_back(n, topdown_data, topdown_time);

        auto two_time = timeSort<postorder<2>>(integers);
        auto two_data = monitorSort<postorder<2>>(integers);
        two.emplace_back(n, two_data, two_time);

        auto three_time = timeSort<postorder<3>>(integers);
        auto three_data = monitorSort<postorder<3>>(integers);
        three.emplace_back(n, three_data, three_time);

        auto four_time = timeSort<postorder<4>>(integers);
        auto four_data = monitorSort<postorder<4>>(integers);
        four.emplace_back(n, four_data, four_time);

        auto five_time = timeSort<postorder<5>>(integers);
        auto five_data = monitorSort<postorder<5>>(integers);
        five.emplace_back(n, five_data, five_time);
    }
    write(two, output_directory+"/postorder_two");
    write(three, output_directory+"/postorder_three");
    write(four, output_directory+"/postorder_four");
    write(five, output_directory+"/postorder_five");
    write(standard, output_directory+"/postorder_standard");
    write(topdown, output_directory+"/postorder_topdown");
}

void testDijkstraCompare() {
    std::vector<std::tuple<int, process_data,long>> postorder_memory_efficient;
    std::vector<std::tuple<int, process_data,long>> topdown_memory_efficient;
    std::vector<std::tuple<int, process_data,long>> standard;

    std::ifstream v_stream("/home/nkb/Documents/Data/Denmark/nodes.txt", std::ios::in);
    std::ifstream e_stream("/home/nkb/Documents/Data/Denmark/edges.txt", std::ios::in);
    Graph graph = Graph(v_stream, e_stream);
    
    auto vertices = generateRandom(2*queries, 0 , graph.vertices() - 1);

    for (int query = 0; query < queries; query++) {
        print_line("\t Query: "+std::to_string(query));
        auto source = vertices.back();
        vertices.pop_back();
        auto target = vertices.back();
        vertices.pop_back();

        int rank = determineRank(graph, source, target);
        if (rank == -1) // Skip instances with no path, saves time.
            continue;


        auto postorder_memory_time = timePostorderDijkstra(graph, source, target);
        auto postorder_memory_data = monitorPostorderDijkstra(graph, source, target);
        postorder_memory_efficient.emplace_back(rank, postorder_memory_data, postorder_memory_time);

        auto topdown_memory_time = timeBinaryDijkstra(graph, source, target);
        auto topdown_memory_data = monitorBinaryDijkstra(graph, source, target);
        topdown_memory_efficient.emplace_back(rank, topdown_memory_data, topdown_memory_time);

        auto standard_time = timeStdDijkstra(graph, source, target);
        auto standard_data = monitorStdDijkstra(graph, source, target);
        standard.emplace_back(rank, standard_data, standard_time);

    }
    write(topdown_memory_efficient, output_directory+"/topdown_memory_efficient_dijkstra_compare");
    write(postorder_memory_efficient, output_directory+"/postorder_memory_efficient_dijkstra_compare");
    write(standard, output_directory+"/standard_dijkstra_compare");

}

int main() {
    print_line("Testing random integers postorder heaps with varying degrees");
    testPostorder();

    print_line("Testing decreasing integers postorder heaps with varying degrees");
    testPostorderSorted(less, "_decreasing");

    print_line("Testing increasing integers dary heaps with varying degrees");
    testPostorderSorted(greater, "_increasing");

    print_line("Compare Dijkstra w. Queues");
    testDijkstraCompare();

    print_line("Finished");
}