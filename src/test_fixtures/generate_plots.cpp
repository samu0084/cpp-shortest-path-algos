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

//17404->30875
//48454->34295

const int source = 48454; // Denmark source: 1699678
const int target = 34295; // Denmark target: 659145
int number_of_landmarks_to_select = 30;
int number_of_active_landmarks_to_pick = 3;
const std::string input_dir = "C:/Users/sx5/CLionProjects/ATB/resources/malta";
const std::string output_dir = input_dir;

typedef std::pair<std::list<int>, std::list<Edge>> draw_data;
typedef std::tuple<std::list<int>, std::list<Edge>, std::list<Edge>> bidirectional_draw_data;

void write(std::vector<int> landmarks, std::string file) {
    std::ofstream output_file(output_dir+"/"+file, std::ofstream::trunc);
    for (auto landmark_id : landmarks) {
        output_file << std::to_string(landmark_id) << '\n';
    }
    output_file.close();
}

void write(draw_data data, std::string file) {
    std::ofstream path_output_file(output_dir+"/"+file+"_path", std::ofstream::trunc);
    std::ofstream edge_order_output_file(output_dir+"/"+file+"_edge_order", std::ofstream::trunc);
    for (auto vertex : data.first) {
        path_output_file << std::to_string(vertex) << '\n';
    }
    for (auto edge : data.second) {
        edge_order_output_file << std::to_string(edge.first) << "," << std::to_string(edge.second) << '\n';
    }
    path_output_file.close();
    edge_order_output_file.close();
}

void write(bidirectional_draw_data data, std::string file) {
    std::ofstream path_output_file(output_dir+"/"+file+"_path", std::ofstream::trunc);
    std::ofstream f_edge_order_output_file(output_dir+"/"+file+"_edge_order_forward", std::ofstream::trunc);
    std::ofstream r_edge_order_output_file(output_dir+"/"+file+"_edge_order_backward", std::ofstream::trunc);
    for (auto vertex : std::get<0>(data)) {
        path_output_file << std::to_string(vertex) << '\n';
    }
    for (auto edge : std::get<1>(data)) {
        f_edge_order_output_file << std::to_string(edge.first) << "," << std::to_string(edge.second) << '\n';
    }
    for (auto edge : std::get<2>(data)) {
        r_edge_order_output_file << std::to_string(edge.first) << "," << std::to_string(edge.second) << '\n';
    }
    path_output_file.close();
    f_edge_order_output_file.close();
    r_edge_order_output_file.close();
}

bidirectional_draw_data generateBidirectionalDijkstra(Graph& graph) {
    BidirectionalAStar algorithm([](int id) { return 0; }, [](int id) { return 0; });
    auto data = algorithm.run(graph, source, target);
    auto path = algorithm.extractPath(std::get<2>(data), std::get<3>(data), std::get<4>(data));
    auto f_edge_order = std::get<6>(data);
    auto r_edge_order = std::get<7>(data);
    return {path, f_edge_order, r_edge_order};
}

bidirectional_draw_data generateCH(Graph& graph) {
    ContractionHierarchies contraction_hierarchy;
    contraction_hierarchy.createContractionHierarchy(graph);
    auto data = contraction_hierarchy.query(source, target);
    auto path = contraction_hierarchy.extractPath(std::get<2>(data), std::get<3>(data), std::get<4>(data));
    auto f_edge_order = std::get<8>(data);
    auto r_edge_order = std::get<9>(data);
    return {path, f_edge_order, r_edge_order};
}

draw_data generateDijkstra(Graph& graph) {
    AStar algorithm([](int id) { return 0; });
    auto data = algorithm.run(graph, source, target);
    auto path = algorithm.extractPath(std::get<1>(data), target);
    auto edge_order = std::get<3>(data);
    return {path, edge_order};
}

draw_data generateLandmarks(Graph& graph, Landmark landmark) {
    AStar algorithm([&landmark](int id) { return landmark.apply(id); });
    landmark.setDestination(target);
    auto data = algorithm.run(graph, source, target);
    auto path = algorithm.extractPath(std::get<1>(data), target);
    auto edge_order = std::get<3>(data);
    return {path, edge_order};
}

draw_data generateReachAStarLandmarks(Graph& graph, Landmark landmark, Reach reach) {
    AStarReach algorithm([&landmark](int id) { return landmark.apply(id); }, reach);
    landmark.setDestination(target);
    auto data = algorithm.run(graph, source, target);
    auto path = algorithm.extractPath(std::get<1>(data), target);
    auto edge_order = std::get<3>(data);
    return {path, edge_order};
}

draw_data generateReachDijkstraLandmarks(Graph& graph, Landmark landmark, Reach reach) {
    AStarReach algorithm([&landmark](int id) { return landmark.apply(id); }, reach);
    landmark.setDestination(target);
    auto data = algorithm.runWithoutExternalHeuristic(graph, source, target);
    auto path = algorithm.extractPath(std::get<1>(data), target);
    auto edge_order = std::get<3>(data);
    return {path, edge_order};
}

draw_data generateAStar(Graph& graph) {
    AStar algorithm([&graph](int id) {
        return (int) std::ceil(computeWeight(
                graph.getCoordinates(id),graph.getCoordinates(target)));
    });
    auto data = algorithm.run(graph, source, target);
    auto path = algorithm.extractPath(std::get<1>(data), target);
    auto edge_order = std::get<3>(data);
    return {path, edge_order};
}

draw_data generateReachAStarEuclideanDistance(Graph& graph, Reach reach) {
    AStarReach algorithm([&graph](int id) {
        return (int) std::ceil(computeWeight(
                graph.getCoordinates(id),graph.getCoordinates(target)));
    }, reach);
    auto data = algorithm.run(graph, source, target);
    auto path = algorithm.extractPath(std::get<1>(data), target);
    auto edge_order = std::get<3>(data);
    return {path, edge_order};
}

draw_data generateReachDijkstraEuclideanDistance(Graph& graph, Reach reach) {
    AStarReach algorithm([&graph](int id) {
        return (int) std::ceil(computeWeight(
                graph.getCoordinates(id),graph.getCoordinates(target)));
    }, reach);
    auto data = algorithm.runWithoutExternalHeuristic(graph, source, target);
    auto path = algorithm.extractPath(std::get<1>(data), target);
    auto edge_order = std::get<3>(data);
    return {path, edge_order};
}

std::vector<int> randomVertices(int number_of_vertices, int from, int to) {
    // First create an instance of an engine.
    std::random_device rnd_device;
    // Specify the engine and distribution.
    std::mt19937 mersenne_engine {rnd_device()};  // Generates random integers
    std::uniform_int_distribution<int> dist {from, to};

    auto gen = [&dist, &mersenne_engine](){
        return dist(mersenne_engine);
    };

    std::vector<int> vector(number_of_vertices);
    generate(begin(vector), end(vector), gen);
    return vector;
}

std::vector<int> randomElementsFromList(std::vector<int> list, int number_of_elements_to_pick) {
    std::random_device rnd_device;
    std::mt19937 mersenne_engine {rnd_device()};
    std::uniform_int_distribution<std::mt19937::result_type> dist(0, list.size() - 1);
    auto gen = [&dist, &mersenne_engine](){
        return dist(mersenne_engine);
    };
    std::vector<int> vector;
    for (int i = 0; i < number_of_elements_to_pick; ++i) {
        vector.emplace_back(list[dist(mersenne_engine)]);
    }
    return vector;
}

int main() {
    std::cout << "Load graph" << std::endl;
    std::ifstream v_stream(input_dir + "/nodes.txt", std::ios::in);
    std::ifstream e_stream(input_dir + "/edges.txt", std::ios::in);
    Graph graph(v_stream, e_stream);

    std::cout << "Load reach" << std::endl;
    std::string reach_bounds_path = input_dir+"/reach_bounds_12000.txt";
    std::ifstream reach_stream(reach_bounds_path, std::ios::in);
    std::vector<int> reach_bounds;
    Reach::read(reach_stream, reach_bounds);
    reach_stream.close();
    Reach reach(reach_bounds);

    std::cout << "Select landmarks" << std::endl;
    Landmark landmark(graph);
    std::vector<int> selected_landmarks = landmark.selectFarthestByShortestPaths(number_of_landmarks_to_select);
    std::cout << "Generate landmarks" << std::endl;
    landmark.generate(selected_landmarks);
    std::cout << "Pick active landmarks: ";
    //std::vector<int> picked_landmarks = landmark.pick(graph.getCoordinates(source), graph.getCoordinates(target), number_of_active_landmarks_to_pick);
    std::vector<int> picked_landmarks = randomElementsFromList(selected_landmarks, number_of_active_landmarks_to_pick);
    for (int id : picked_landmarks) {
        std::cout << id << ", ";
    }
    std::cout << std::endl;
    landmark.activate(picked_landmarks);
    std::string select_pick = std::to_string(number_of_landmarks_to_select)+"_"+std::to_string(number_of_active_landmarks_to_pick);
    write(selected_landmarks, "selected_landmarks_"+select_pick);
    write(picked_landmarks, "active_landmarks_"+select_pick);

    std::cout << "Dijkstra \n";
    write(generateDijkstra(graph), "dijkstra");
    std::cout << "AStar \n";
    write(generateAStar(graph), "a_star");
    std::cout << "Landmarks \n";
    write(generateLandmarks(graph, landmark), "landmarks_"+select_pick);
    std::cout << "Bidirectional Dijkstra\n";
    write(generateBidirectionalDijkstra(graph), "bidirectional_dijkstra");
    std::cout << "Reach AStar Euclidean Distance\n";
    write(generateReachAStarEuclideanDistance(graph, reach), "reach_a_star_euclidean_distance");
    std::cout << "Reach Dijkstra Euclidean Distance\n";
    write(generateReachDijkstraEuclideanDistance(graph, reach), "reach_dijkstra_euclidean_distance");
    std::cout << "Reach AStar Landmarks \n";
    write(generateReachAStarLandmarks(graph, landmark, reach), "reach_a_star_landmarks_"+select_pick);
    std::cout << "Reach Dijkstra Landmarks\n";
    write(generateReachDijkstraLandmarks(graph, landmark, reach), "reach_dijkstra_landmarks_"+select_pick);
    std::cout << "CH \n";
    write(generateCH(graph), "contraction_hierarchies");
}