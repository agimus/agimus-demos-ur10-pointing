#include "tsp.hpp"

namespace tsp {
ordered_neighbors_t neighborMatrix(const distance_matrix_t& d)
{
  auto n (d.rows());
  ordered_neighbors_t neighbors (n-1, n);
  for (int i = 0; i < n; ++i) {
    // Sort neighbors k by d(k, i)
    for (int k = 0, j = 0; k < n; ++k)
      if (k!=i) neighbors(j++, i) = k;
    std::sort(neighbors.col(i).data(), neighbors.col(i).data()+n-1,
        [&d,&i](auto u, auto v) { assert(i!=u && i!=v); return d(u,i) < d(v,i); });
  }
  return neighbors;
}

namespace heuristic_nearest {
std::tuple<double, path_t> solve (const distance_matrix_t& d)
{
  if (d.cols() != d.rows())
    throw std::invalid_argument("expect a square distance matrix");
  auto n = d.cols();

  path_t sol (n-1);
  std::vector<int> remaining (n-1);
  for (int ic = 1; ic < n; ++ic)
    remaining[ic-1] = ic;
  std::vector<bool> visited(n-1, false);
  int jc = 0;
  double cost = 0;
  for (int i = 1; i < n; ++i) {
    double dmin = std::numeric_limits<double>::infinity();
    std::vector<int>::iterator _kmin;
    for (auto _k = remaining.begin(); _k != remaining.end(); ++_k) {
      int kc = *_k;
      if (d(jc,kc) < dmin) {
        dmin = d(jc,kc);
        _kmin = _k;
      }
    }
    jc = sol[i-1] = *_kmin;
    std::swap(*_kmin, *remaining.rbegin());
    remaining.resize(remaining.size()-1);
    cost += dmin;
  }
  cost += d(sol.back(), 0);
  return std::make_tuple(cost, sol);
}
}
namespace dynamic_programming {

typedef uint64_t city_set_t;
typedef std::vector<path_t> cache_t;

// Step 1: get minimum distance
// Compute the distance to go from city 0, visiting all cities in N and finishing at i \notin N
// \param dmin the minimal distance between two different cities.
// \param i final city.
// \param N an integer with one bit per city, indicating whether it is in the set.
// \param nN the number of cities in the set (for optimization only, redundant with N).
// \param depth should be d.rows() - nN (-1?)
// \param costToCome
double dist (const distance_matrix_t& d,
    const ordered_neighbors_t& neighbors,
    const double& dmin,
    int i, city_set_t N, int nN,
    cache_t& pathCache,
    int depth,
    double costToCome,
    double& costUpperBound)
{
  if (nN == 0) {
    assert(depth < pathCache.size());
    pathCache[depth][0] = i;
    return d(0, i);
  }

  // Store the costs in the form (nj, dist(nj, N))
  city_set_t NN;
  double minCost = std::numeric_limits<double>::infinity();
  int k = 0;
  assert(d.cols() == neighbors.rows()+1);
  for (int in = 0; in < neighbors.rows(); ++in) { // Loop on the neighbors by order of d(k, i)
    int j = neighbors(in, i);
    if (!(N & (1 << j))) continue;
    assert(j != i);
    if (costToCome + d(j,i) + (nN-1) * dmin < costUpperBound) {
      NN = N & ~(1 << j);
      double cost = dist(d, neighbors, dmin, j, NN, nN-1, pathCache, depth+1, costToCome + d(j,i), costUpperBound) + d(j,i);
      if (cost < minCost) {
        minCost = cost;
        //bestSubpath.swap(bestPath);
        //bestPath[nN]= i;
        assert(depth+1 < pathCache.size());
        pathCache[depth+1].swap(pathCache[depth]);
        if (i > 0) { // i == 0 for the first call.
          assert(nN < pathCache[depth].size());
          pathCache[depth][nN]= i;
        }

        if (costToCome + cost < costUpperBound)
          costUpperBound = costToCome + cost;
      }
    }
    if (++k == nN) break;
  }
  return minCost;
}

std::tuple<double, path_t> solveWithBound (const distance_matrix_t& d,
    double costUpperBound)
{
  if (d.cols() != d.rows())
    throw std::invalid_argument("expect a square distance matrix");
  auto n = d.cols();
  if (n > 64)
    throw std::invalid_argument("at the moment, cannot solve instance with more that 64 cities.");

  constexpr city_set_t all (~city_set_t(0));
  city_set_t N = (all >> (64-n)) & (all << 1);

  ordered_neighbors_t neighbors(neighborMatrix(d));

  //path_t bestPath (n-1);
  //double cost = dist(d, 0, N, n-1, bestPath);
  //return std::make_tuple(cost, bestPath);
  cache_t pathCache (n, path_t(n-1));
  //cache_t pathCache (n, n-1);
  double dmin = std::numeric_limits<double>::infinity();
  for (int i = 1; i < d.rows(); ++i) {
    dmin = std::min({
        d.row(i).head(i).minCoeff(),
        d.row(i-1).tail(d.rows()-i).minCoeff(),
        dmin});
  }
  double cost = dist(d, neighbors, dmin, 0, N, n-1, pathCache, 0, 0, costUpperBound);
  return std::make_tuple(cost, pathCache[0]);
}

std::tuple<double, path_t> solveWithHeuristic (const distance_matrix_t& d)
{
  double costUpperBound;
  path_t path;
  std::tie(costUpperBound, path) = heuristic_nearest::solve(d);
  return solveWithBound(d, costUpperBound);
}
}
namespace brute_force {
std::tuple<double, path_t> solve (const distance_matrix_t& d)
{
  if (d.cols() != d.rows())
    throw std::invalid_argument("expect a square distance matrix");
  auto n = d.cols();
  if (n > 64)
    throw std::invalid_argument("at the moment, cannot solve instance with more that 64 cities.");

  path_t current (n-1), bestPath;
  double minCost = -1;
  for (int i = 0; i < n-1; ++i) current[i] = i+1;
  do {
    double cost = d(0, current[0]);
    for (int i = 0; i < n-2; ++i)
      cost += d(current[i], current[i+1]);
    cost += d(current.back(), 0);
    if (minCost < 0 || cost < minCost) {
      minCost = cost;
      bestPath = current;
    }
  } while (std::next_permutation(current.begin(), current.end()));

  return std::make_tuple(minCost, bestPath);
}
}
}

// vim: foldmethod=syntax
