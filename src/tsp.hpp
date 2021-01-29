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
std::tuple<double, path_t> solveWithBound (const distance_matrix_t& d,
    double costUpperBound,
    bool allowPruning = true);

inline std::tuple<double, path_t> solve (const distance_matrix_t& d,
    bool allowPruning = true)
{
  return solveWithBound(d, std::numeric_limits<double>::infinity(), allowPruning);
}

std::tuple<double, path_t> solveWithHeuristic (const distance_matrix_t& d,
    bool allowPruning = true);

}
namespace brute_force {
std::tuple<double, path_t> solve (const distance_matrix_t& d);
}
namespace approximative_kopt {
void swap2opt(path_t& path, int i, int k);
bool swap3opt(const distance_matrix_t& d, path_t& path, int i, int j, int k);
std::tuple<double, path_t> solve2opt (const distance_matrix_t& d, path_t initialGuess = path_t());
std::tuple<double, path_t> solve3opt (const distance_matrix_t& d, path_t initialGuess = path_t());
}
}
