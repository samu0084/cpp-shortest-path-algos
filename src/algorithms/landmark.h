#ifndef ATB_LANDMARK_H
#define ATB_LANDMARK_H

#include <map>
#include "a_star.h"
#include <fstream>
#include <random>
#include "../../modules/post-order-heap/src/post-order_heap.hpp"
#include "../graph/graph.h"
#include <queue>
#include <climits>
#include "bfs.h"
#include <cmath>

struct Point {
    explicit Point(std::pair<double, double> coordinates) : y_(coordinates.second), x_(coordinates.first) {}
    //explicit Point(double x, double y) : y_(y), x_(x) {}
    double x_;
    double y_;
};

struct Line {

    Line(double m, double b) : m_(m), b_(b) {}
    double m_;
    double b_;
};

struct LandmarkAngle {
    LandmarkAngle(int landmark, double angle) : landmark_(landmark), angle_(angle) {}
    int landmark_;
    double angle_;
};

enum RelativePosition {
    before,
    between,
    after
};

class Landmark {
public:
    /**
     * @brief Landmark constructor.
     * Creates a Landmark-object, based on the referenced graph.
     * @param graph
     * @param reverse
     */
    Landmark(Graph& graph);

    ~Landmark();

    [[nodiscard]] const std::vector<std::pair<int, WGS84>> &getAllLandmarks() const;

    [[nodiscard]] const std::vector<std::pair<std::vector<int>&, std::vector<int>&>>& getActiveLandmarks() const;

    /**
     * @brief Generates landmarks for the specified vertex ids.
     * The landmark distances are computed by running Dijkstra (A* with 0-heuristic) on the specified vertices.
     * @param landmark_ids
     */
    void generate(const std::vector<int>& landmark_ids);

    /**
     * @brief Activates previously generated landmarks.
     * The specified landmarks must have been previously generated.
     * @param landmark_ids
     */
    void activate(const std::vector<int>& landmark_ids);

    /**
     * @brief Computes the %count most suited vertices for landmarks ...
     *
     * @param count
     */
    std::vector<int> selectFarthestByShortestPaths(int count);

    /**
     * @brief Computes the %count most suited vertices for landmarks ...
     *
     * @param count
     */
    std::vector<int> selectFarthestByEuclideanDistance(int count);

    /**
     * @brief Computes the %count most suited vertices for landmarks ...
     *
     * @param count
     */
    std::vector<int> selectFarthestByBFS(int count);

    /**
     * @brief Deactivates all active landmarks.
     */
    void deactivate();

    /**
     * @brief Computes a lower-bound of the distance from the destination to the specified vertex.
     * The destination id must be set, and the vertex specified by id must exist in the graph.
     * The lower-bound is computed using the triangle inequality.
     * @param id
     * @return inting-point lower-bound.
     */
    int apply(int id);

    /**
     * @brief sets the vertex destination id.
     * @param new_destination
     */
    void setDestination(int new_destination);
    std::vector<int> pick(WGS84 origin, WGS84 destination, int number_of_landmarks_to_pick);
private:
    // Reference to graph.
    Graph& graph_;
    // All landmarks;
    std::vector<std::pair<int, WGS84>> all_landmarks_;
    // Map of id -> files containing computed distances. Stores active & inactive landmarks.
    std::map<int, std::pair<std::string, std::string>> files_;
    // List of currently active landmarks.
    std::vector<std::pair<std::vector<int>&, std::vector<int>&>> active_landmarks_;
    // List of landmark forward, backward computed distances
    std::map<int, std::pair<std::vector<int>, std::vector<int>>> all_landmark_distances_;

    // Vertex-id of the destination. Used for lower-bound computations.
    int destination_;
    typedef std::vector<int> CalculateAllDistances(Graph &graph, int vertex);

    /**
     * @brief Computes the %count most suited vertices for landmarks ...
     */
    std::vector<int> selectFarthest(int count, CalculateAllDistances calculator_all_distances);

    static Point WGS84ToPoint(WGS84 location);

    static Line makeLine(Point p1, Point p2);

    static Line makeNormal(Line line, Point though_point);

    static RelativePosition relativePosition(Point vertex_point, Line line, Point point_on_line);

    static RelativePosition relativePosition(Point vertex_point, Line first, Point point_on_first, Line second, Point point_on_second);

    static double leastAngle(Line line_one, Line line_two);

    friend class Landmark_LeastAngle_Test;
    friend class Landmark_relativePositionRealGraph_Test;

    static Point computeMercatorProjection(WGS84 coordinates);




};


#endif //ATB_LANDMARK_H