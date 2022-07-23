#include "landmark.h"

Landmark::Landmark(Graph &graph) : graph_(graph), destination_(0) {

}

void Landmark::generate(const std::vector<int>& landmark_ids) {
    AStar dijkstra([](int id) {return 0;});
    for (const auto& id : landmark_ids) {
        Data forward_data = dijkstra.run(graph_, id, -1);
        Data reverse_data = dijkstra.reverse(graph_, id, -1);
        all_landmark_distances_[id] =  std::pair<std::vector<int>, std::vector<int>>(std::get<0>(forward_data), std::get<0>(reverse_data));
    }
    all_landmarks_.clear();
    for (int i = 0; i < landmark_ids.size(); ++i) {
        all_landmarks_.emplace_back(landmark_ids[i],graph_.getCoordinates(landmark_ids[i]));
    }
}

void Landmark::activate(const std::vector<int>& landmark_ids) {
    active_landmarks_.clear();
    for (auto id : landmark_ids) {
        active_landmarks_.emplace_back(all_landmark_distances_[id].first, all_landmark_distances_[id].second);
    }
}

void Landmark::deactivate() {
    active_landmarks_.clear();
}

int Landmark::apply(int id) {
    int best_bound = 0;
    int infinity = std::numeric_limits<int>::max();
    for(auto distances : active_landmarks_){
        auto& forward = distances.first;
        auto& reverse = distances.second;
        int forward_delta = forward[destination_] - forward[id];
        int reverse_delta = reverse[id] - reverse[destination_];
        int lower_bound = std::max(forward_delta,reverse_delta);
        if(lower_bound > best_bound && lower_bound < infinity){
            best_bound = lower_bound;
        }
    }
    return best_bound;
}

std::vector<int> Landmark::selectFarthestByShortestPaths(int count){
    static auto calculate_all_distances = [](Graph &graph, int vertex) {
        AStar dijkstra([](int vertex)-> int {return 0;});
        int all_destinations = -1;
        return std::get<0>(dijkstra.run(graph, vertex, all_destinations));
    };
    return selectFarthest(count, calculate_all_distances);
}

std::vector<int> Landmark::selectFarthestByEuclideanDistance(int count){
    static auto calculate_all_distances = [](Graph &graph, int vertex) {
        std::vector<int> euclidean_distances(graph.vertices());
        WGS84 from = graph.getCoordinates(vertex);
        for (int i = 0; i < graph.vertices(); ++i) {
            WGS84 to = graph.getCoordinates(i);
            euclidean_distances[i] = (int) computeWeight(from, to);
        }
        return euclidean_distances;
    };
    return selectFarthest(count, calculate_all_distances);
}

std::vector<int> Landmark::selectFarthestByBFS(int count) {
    static auto calculate_all_distances = [](Graph &graph, int vertex) { ;
        auto result = BFS::run(graph, vertex).first;
        return result;
    };
    return selectFarthest(count, calculate_all_distances);
}

int indexOfGreatest(std::vector<int> vector) {
    int indexGreatest = -1;
    int max = INT_MIN;
    for(int i =0; i < vector.size(); i++) {
        if (vector[i] > max && vector[i] < INT_MAX) {
            max = vector[i];
            indexGreatest = i;
        }
    }
    return indexGreatest;
}

std::vector<int> Landmark::selectFarthest(int count, CalculateAllDistances calculate_all_distances){
    /* ±| ubs
     * Pick a start vertex at random
     * and find the  vertex that is the farthest away from it
     * .Add v to the set of landmarks.Proceed in iterations,always adding to these the vertex that is farthest from it.
     */
    std::random_device random_device;
    std::mt19937 random_number_generator(random_device());
    std::uniform_int_distribution<std::mt19937::result_type> distribution(0, graph_.vertices() - 1); // distribution in range [n, number_of_nodes]

    std::vector<int> landmarks(count);
    int random_node = distribution(random_number_generator);
    std::vector<int> node_distances = calculate_all_distances(graph_, random_node);
    landmarks[0] = indexOfGreatest(node_distances);
    std::vector<int> combined_distances = calculate_all_distances(graph_, landmarks[0]);
    for(int i = 1; i < count; ++i){
        landmarks[i] = indexOfGreatest(combined_distances);
        node_distances = calculate_all_distances(graph_, landmarks[i]);
        for (int j = 0; j < node_distances.size(); ++j) {
            if (combined_distances[j] > node_distances[j]) {
                combined_distances[j] = node_distances[j];
            }
        }
    }
    return landmarks;
}

void Landmark::setDestination(int new_destination) {
    destination_ = new_destination;
}



Line Landmark::makeLine(Point p1, Point p2) {
    double m = (p2.y_ - p1.y_) / (p2.x_ - p1.x_);
    double b = p1.y_ - m * p1.x_;
    Line line(m, b);
    return line;
}

Line Landmark::makeNormal(Line line, Point though_point) {
    double m_normal = -1 / line.m_;
    double b_normal = though_point.y_ - m_normal * though_point.x_;
    Line normal(m_normal, b_normal);
    return normal;
}

RelativePosition Landmark::relativePosition(Point vertex_point, Line line, Point point_on_line) {
    if (line.m_ == INFINITY) {
        return point_on_line.x_ > vertex_point.x_ ? RelativePosition::before : RelativePosition::after;
    }
    double y_calculated = line.m_ * vertex_point.x_ + line.b_;
    if (line.m_ > 0) {
        return y_calculated < vertex_point.y_ ? RelativePosition::before : RelativePosition::after;
    } else {
        return y_calculated >= vertex_point.y_ ? RelativePosition::before : RelativePosition::after;
    }
}


RelativePosition Landmark::relativePosition(Point vertex_point, Line first, Point point_on_first, Line second, Point point_on_second) {
    RelativePosition relative_to_first = relativePosition(vertex_point, first, point_on_first);
    RelativePosition relative_to_second = relativePosition(vertex_point, second, point_on_second);
    // From bottom to top is considered the same as left to right
    bool left_to_right;
    if (first.m_ == 0) {
        left_to_right = first.b_ > second.b_;
    } else {
        double x_for_y_zero_on_first = -first.b_/first.m_;
        double x_for_y_zero_on_second = -second.b_/second.m_;
        left_to_right = x_for_y_zero_on_first < x_for_y_zero_on_second;
    }
    if (left_to_right) { // Left to right
        if (relative_to_first == RelativePosition::before) return RelativePosition::before;
        if (relative_to_second == RelativePosition::after) return RelativePosition::after;
    } else { // Right to leaf
        if (relative_to_first == RelativePosition::after) return RelativePosition::after;
        if (relative_to_second == RelativePosition::before) return RelativePosition::before;
    }
    return RelativePosition::between;
}


double Landmark::leastAngle(Line line_one, Line line_two) {
    auto res = std::abs(atan((line_one.m_-line_two.m_)/(1+line_one.m_*line_two.m_)));
    return res;

}

class CompareAngle
{
public:
    bool operator() (LandmarkAngle o1, LandmarkAngle o2)
    {
        return o1.angle_ > o2.angle_;
    }
};

std::vector<int> Landmark::pick(WGS84 origin, WGS84 destination, int number_of_landmarks_to_pick) {
    if (number_of_landmarks_to_pick > all_landmarks_.size()) {
        number_of_landmarks_to_pick = all_landmarks_.size();
    }
    Point origin_point = WGS84ToPoint(origin);
    Point destination_point = WGS84ToPoint(destination);

    Line line_origin_destination = makeLine(origin_point, destination_point);
    Line normal_through_origin = makeNormal(line_origin_destination, origin_point);
    Line normal_through_destination = makeNormal(line_origin_destination, destination_point);

    std::priority_queue<LandmarkAngle, std::vector<LandmarkAngle>, CompareAngle> angles_before;
    std::priority_queue<LandmarkAngle, std::vector<LandmarkAngle>, CompareAngle> angles_after;
    std::priority_queue<LandmarkAngle, std::vector<LandmarkAngle>, CompareAngle> angles_between;
    for(auto [landmark, distances] : all_landmarks_) {
        Point landmark_point = WGS84ToPoint(graph_.getCoordinates(landmark));
        RelativePosition position = relativePosition(landmark_point, normal_through_origin, origin_point, normal_through_destination, destination_point);
        double angle;
        switch (position) {
            case RelativePosition::before: {
                Line line_origin_landmark = makeLine(origin_point, WGS84ToPoint(graph_.getCoordinates(landmark)));
                angle = leastAngle(line_origin_destination, line_origin_landmark);
                angles_before.emplace(landmark, angle);
                break;
            }
            case RelativePosition::between: {
                angle = 0;
                angles_between.emplace(landmark, 0);
                break;
            }
            case RelativePosition::after: {
                Line line_destination_landmark = makeLine(destination_point, WGS84ToPoint(graph_.getCoordinates(landmark)));
                angle = leastAngle(line_origin_destination, line_destination_landmark);
                angles_after.emplace(landmark, angle);
                break;
            }
        }
    }
    // Pick one landmark from before and one landmark from after
    std::vector<int> picked_landmarks;
    if (!angles_before.empty() && picked_landmarks.size() < number_of_landmarks_to_pick) {
        picked_landmarks.push_back(angles_before.top().landmark_);
        angles_before.pop();
    }
    if (!angles_after.empty() && picked_landmarks.size() < number_of_landmarks_to_pick) {
        picked_landmarks.push_back(angles_after.top().landmark_);
        angles_after.pop();
    }
    // Pick remaining from before and after
    while (!angles_before.empty() && !angles_after.empty() && picked_landmarks.size() < number_of_landmarks_to_pick) {
        LandmarkAngle before = angles_before.top();
        LandmarkAngle after = angles_after.top();
        if (before.angle_ < after.angle_) {
            angles_before.pop();
            picked_landmarks.push_back(before.landmark_);
        } else {
            angles_after.pop();
            picked_landmarks.push_back(after.landmark_);
        }
    }
    while (!angles_before.empty() && picked_landmarks.size() < number_of_landmarks_to_pick) {
        LandmarkAngle before = angles_before.top();
        angles_before.pop();
        picked_landmarks.push_back(before.landmark_);
    }
    while (!angles_after.empty() && picked_landmarks.size() < number_of_landmarks_to_pick) {
        LandmarkAngle after = angles_after.top();
        angles_after.pop();
        picked_landmarks.push_back(after.landmark_);
    }
    // Pick remaining from between
    while (picked_landmarks.size() < number_of_landmarks_to_pick) {
        picked_landmarks.push_back(angles_between.top().landmark_);
        angles_between.pop();
    }
    return picked_landmarks;
}

double convertToRadians(double degree) {
    double pi = M_PI;
    return (degree * (pi / 180));
}

Point Landmark::computeMercatorProjection(WGS84 coordinates) {
    double latitudeInRadians =  convertToRadians(coordinates.first);
    double longitudeInRadians = convertToRadians(coordinates.second);
    double t1 = 256 / (2 * M_PI);
    double t2 = t1 * pow(2, 10);
    double x =  t2 * (longitudeInRadians + M_PI);
    double y =  t2 * (M_PI-log(tan(M_PI / 4 + latitudeInRadians / 2)));
    Point point({x, y});
    return point;
}

Point Landmark::WGS84ToPoint(WGS84 location) {
    Point point(computeMercatorProjection(location));
    return point;
}

const std::vector<std::pair<int, WGS84>> &Landmark::getAllLandmarks() const {
    return all_landmarks_;
}

const std::vector<std::pair<std::vector<int>&, std::vector<int>&>>& Landmark::getActiveLandmarks() const {
    return active_landmarks_;
}

Landmark::~Landmark() = default;
