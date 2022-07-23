//
// Created by nicolaj on 3/12/22.
//

#include "graph.h"

Graph::Vertex::Vertex() = default;

Graph::Vertex::Vertex(int id, float lat, float lon) : id_(id), lat_(lat), lon_(lon) { }

void Graph::Vertex::addOutgoing(int id, int weight) {
    outgoing_.emplace_back(id, weight);
}

void Graph::Vertex::addIncoming(int id, int weight) {
    incoming_.emplace_back(id, weight);
}

std::vector<Edge>& Graph::Vertex::getOutgoing() {
    return outgoing_;
}

std::vector<Edge>& Graph::Vertex::getIncoming() {
    return incoming_;
}

WGS84 Graph::Vertex::getCoordinates() {
    return {lat_, lon_};
}

WGS84 Graph::getCoordinates(int vertex_id) {
    return vertices_[vertex_id].getCoordinates();
}

int Graph::Vertex::getId() {
    return id_;
}

std::vector<Edge>& Graph::getOutgoing(int vertex_id) {
    return vertices_[vertex_id].getOutgoing();
}

std::vector<Edge>& Graph::getIncoming(int vertex_id) {
    return vertices_[vertex_id].getIncoming();
}

void Graph::createEdge(int source, int target, int weight) {
    vertices_[source].addOutgoing(target, weight);
    vertices_[target].addIncoming(source, weight);
    ++number_of_edges_;
}

void Graph::removeEdge(int source, int target) {
    auto& edges_from_source = getOutgoing(source);
    auto& edges_to_target = getIncoming(target);

    // Use Erase-remove idiom for deletion ...
    bool edge_deleted = false;
    auto new_end_out = std::remove_if(edges_from_source.begin(), edges_from_source.end(), [target](Edge edge){return edge.first == target;});
    edge_deleted = new_end_out - edges_from_source.end() != 0;
    edges_from_source.erase(new_end_out, edges_from_source.end());

    auto new_end_in = std::remove_if(edges_to_target.begin(), edges_to_target.end(), [source](Edge edge){return edge.first == source;});
    if (!edge_deleted)
        edge_deleted = new_end_out - edges_from_source.end() != 0;
    edges_to_target.erase(new_end_in, edges_to_target.end());
    number_of_edges_ += edge_deleted ? -1 : 0;
}

int Graph::vertices() { return number_of_vertices_; }

int Graph::edges() { return number_of_edges_; }

long count(std::istream &stream) {
    int count = 0;
    std::string line;
    while (stream >> line)
        count++;
    stream.clear();
    stream.seekg(std::ios::beg);
    return count;
}

double toRadians(double degree) {
    double pi = 3.14159265359;
    return (degree * (pi / 180));
}

/**
 * Computes the weight of edges, using a version of Haversine Great Sphere formula adapted for floating point precision.
 * The computed distance is measured in meters.
 * @param origin the integer ID of the origin node.
 * @param destination the integer ID of the destination node.
 * @param nodes a vector containing the lat/lon pairs of all nodes.
 * @return
 */
double computeWeight(WGS84 origin, WGS84 destination) {
    const double RADIUS_OF_EARTH_IN_KILOMETERS = 6378.1;
    const double RADIUS_OF_EARTH_IN_METERS = RADIUS_OF_EARTH_IN_KILOMETERS * 1000;

    const double R = RADIUS_OF_EARTH_IN_METERS;

    //Deltas
    double dLat = toRadians(destination.first - origin.first);
    double dLon = toRadians(destination.second - origin.second);

    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(toRadians(origin.first)) * cos(toRadians(destination.first)) * sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * asin(fmin(1, sqrt(a)));

    double d = R * c;
    return d;
}

Graph::Graph(std::vector<int> &vertices, std::vector<std::tuple<int, int, int>> &edges) {
    // Save vertices ...
    for (auto id : vertices) {
        vertices_.emplace_back(id,0,0);
    }
    number_of_vertices_ = vertices_.size();
    // Sort vertices according to ID.
    auto comparator = [](Vertex left, Vertex right){
        return left.getId() < right.getId();
    };
    std::sort(vertices_.begin(), vertices_.end(), comparator);
    // Save edges
    number_of_edges_ = 0;
    for (auto [origin, destination, weight] : edges) {
        createEdge(origin, destination, weight);
    }
}

Graph::Graph(std::vector<std::tuple<int, float, float>> &vertices, std::vector<std::tuple<int, int>> &edges) {
    // Save vertices ...
    for (auto [id, latitude, longitude] : vertices) {
        vertices_.emplace_back(id,latitude,longitude);
    }
    number_of_vertices_ = vertices_.size();
    // Sort vertices according to ID.
    auto comparator = [](Vertex left, Vertex right){
        return left.getId() < right.getId();
    };
    std::sort(vertices_.begin(), vertices_.end(), comparator);
    // Save edges
    number_of_edges_ = 0;
    for (auto [origin, destination] : edges) {
        Vertex origin_v = vertices_[origin];
        Vertex destination_v = vertices_[destination];
        int weight = (int) std::ceil(computeWeight(origin_v.getCoordinates(), destination_v.getCoordinates()));
        createEdge(origin, destination, weight);
    }
}

Graph::Graph(Graph &other_graph, bool keepEdges) {
    // Add vertices
    number_of_vertices_ = other_graph.number_of_vertices_;
    for (const auto& vertex_other : other_graph.vertices_) {
        vertices_.emplace_back(vertex_other.id_, vertex_other.lat_, vertex_other.lon_);
    }
    // Add edges
    number_of_edges_ = 0;
    if (keepEdges) {
        number_of_edges_ = other_graph.number_of_edges_;
        for (const Vertex& vertex_other : other_graph.vertices_) {
            for (const auto [source, weight] : vertex_other.incoming_) {
                vertices_[vertex_other.id_].addIncoming(source, weight);
            }
            for (const auto [target, weight] : vertex_other.outgoing_) {
                vertices_[vertex_other.id_].addOutgoing(target, weight);
            }
        }
    }
}

Graph::Graph(std::istream& v_stream, std::istream& e_stream) {
    int id;
    long long int oldID;
    float lat, lon;
    char comma;

    // Load vertices ...
    while (v_stream >> id >> comma >> oldID >> comma >> lat >> comma >> lon && (comma == ',')) {
        vertices_.emplace_back(id, lat, lon);
    }
    number_of_vertices_ = vertices_.size();

    // Sort vertices according to ID.
    auto comparator = [](Vertex left, Vertex right){
        return left.getId() < right.getId();
    };
    std::sort(vertices_.begin(), vertices_.end(), comparator);

    // Load edges
    int origin, destination;
    number_of_edges_ = 0;
    while (e_stream >> origin >> comma >> destination && (comma == ',')) {
        Vertex origin_v = vertices_[origin];
        Vertex destination_v = vertices_[destination];
        int weight = (int) std::ceil(computeWeight(origin_v.getCoordinates(), destination_v.getCoordinates()));
        createEdge(origin, destination, weight);
    }
}




