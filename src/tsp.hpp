#include <vector>
#include <tuple>

#include <Eigen/Core>

namespace tsp {
typedef Eigen::MatrixXd distance_matrix_t;
typedef std::vector<int> path_t;
typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> ordered_neighbors_t;

distance_matrix_t randomDistanceMatrix(int N);
/// Compute nearest neighbor from i
ordered_neighbors_t neighborMatrix(const distance_matrix_t& d);
double evaluateCost(const distance_matrix_t& d, const path_t& path);

namespace heuristic_nearest {
std::tuple<double, path_t> solve (const distance_matrix_t& d);
}
namespace dynamic_programming {

typedef uint64_t city_set_t;
typedef std::vector<path_t> cache_t;

std::tuple<double, path_t> solveWithBound (const distance_matrix_t& d,
    double costUpperBound);

inline std::tuple<double, path_t> solve (const distance_matrix_t& d)
{
  return solveWithBound(d, std::numeric_limits<double>::infinity());
}

std::tuple<double, path_t> solveWithHeuristic (const distance_matrix_t& d);

}
namespace brute_force {
std::tuple<double, path_t> solve (const distance_matrix_t& d);
}
}
