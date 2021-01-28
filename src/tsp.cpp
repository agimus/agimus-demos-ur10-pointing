#include "tsp.hpp"

#include <map>

#ifdef TSP_DP_DEBUG
#include <iostream>
#include <iomanip>
#endif

namespace tsp {
distance_matrix_t randomDistanceMatrix(int n) {
  distance_matrix_t d(n,n);
  d.setRandom();
  d.array() += 1;
  d.diagonal().setZero();
  d.triangularView<Eigen::StrictlyUpper>() =
    d.triangularView<Eigen::StrictlyLower>().transpose();
  return d;
}

ordered_neighbors_t neighborMatrix(const distance_matrix_t& d) {
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

double evaluateCost(const distance_matrix_t& d, const path_t& path) {
  double cost = d(0, path.front()) + d(path.back(),0);
  for (int k = 0; k < path.size()-1; ++k)
    cost += d(path[k], path[k+1]);
  return cost;
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
#ifdef TSP_DP_DEBUG
# define TSP_DP_IN_DEBUG(x) x
#else
# define TSP_DP_IN_DEBUG(x)
#endif

typedef uint64_t city_set_t;
typedef std::vector<path_t> cache_t;

struct DP {
  const distance_matrix_t& d;
  const ordered_neighbors_t neighbors;
  double dmin;

  cache_t pathCache;
  double costUpperBound = std::numeric_limits<double>::infinity();

  bool useCachedCosts = true;
  typedef struct { double cost; int nvisits; } cache_type;
  std::map<uint64_t, cache_type> cachedCosts;

#ifdef TSP_DP_DEBUG
  long int ncalls = 0, nroots = 0, nupdates = 0;
#endif

  DP(const distance_matrix_t& _d)
    : d(_d), neighbors(neighborMatrix(d)),
    dmin(neighbors.row(0).minCoeff()), pathCache (d.cols(), path_t(d.cols()-1))
  {}

  // Compute the distance to go from city 0, visiting all cities in N and finishing at i \notin N
  // \param dmin the minimal distance between two different cities.
  // \param i final city.
  // \param N an integer with one bit per city, indicating whether it is in the set.
  // \param nN the number of cities in the set (for optimization only, redundant with N).
  // \param depth should be d.rows() - nN (-1?)
  // \param costToCome
  double dist (int i, city_set_t N, int nN, int depth, double costToCome);
};

double DP::dist (int i, city_set_t N, int nN, int depth, double costToCome)
{
  TSP_DP_IN_DEBUG(++ncalls;)
  if (nN == 0) {
    TSP_DP_IN_DEBUG(++nroots;)
    assert(depth < pathCache.size());
    pathCache[depth][0] = i;
    return d(0, i);
  }
  // Make the key
  uint64_t key = N | (uint64_t(i) << 32);
  decltype(cachedCosts.emplace(key, cache_type())) cachedCost;
  cachedCost = cachedCosts.emplace(key, cache_type{ 0., 0 });
  cachedCost.first->second.nvisits++;
  if (useCachedCosts && !cachedCost.second)//no inserted
    return cachedCost.first->second.cost;

  // Store the costs in the form (nj, dist(nj, N))
  city_set_t NN;
  double minCost = std::numeric_limits<double>::infinity();
  int k = 0;
  assert(d.cols() == neighbors.rows()+1);
  for (int in = 0; in < neighbors.rows(); ++in) { // Loop on the neighbors by increasing order of d(k, i)
    int j = neighbors(in, i);
    if (!(N & (1 << j))) continue;
    assert(j != i);
    if (costToCome + d(j,i) + nN * dmin < costUpperBound) {
      NN = N & ~(1 << j);
      double cost = dist(j, NN, nN-1, depth+1, costToCome + d(j,i)) + d(j,i);
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

        if (costToCome + cost < costUpperBound) {
          costUpperBound = costToCome + cost;
          TSP_DP_IN_DEBUG(nupdates++;)
        }
      }
    }
    if (++k == nN) break;
  }
  cachedCost.first->second.cost = minCost;
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

  DP dp (d);
  dp.costUpperBound = costUpperBound;
  dp.useCachedCosts = true;

  /*
  double dmin = std::numeric_limits<double>::infinity();
  for (int i = 1; i < d.rows(); ++i) {
    dmin = std::min({
        d.row(i).head(i).minCoeff(),
        d.row(i-1).tail(d.rows()-i).minCoeff(),
        dmin});
  }
  */
  double cost = dp.dist(0, N, n-1, 0, 0);
#ifdef TSP_DP_DEBUG
  std::cout << n << ' ' << dp.nupdates << '/' << dp.nroots << '/' << dp.ncalls << std::endl;
  long int total = 0;
  for (auto pair : dp.cachedCosts) {
    int finalcity = pair.first >> 32;
    uint32_t N (pair.first);
    std::cout << std::setfill(' ') << std::setw(5) << N << ' ' << std::setfill('0') << std::setw(8) << std::hex << finalcity << std::dec << ' ' << pair.second.nvisits << '\n';
    total += pair.second.nvisits;
  }
  std::cout << visited.size() << '/' << total << " .....\n";
#endif
  return std::make_tuple(cost, dp.pathCache[0]);
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
namespace approximative_kopt {
void swap2opt(path_t& path, int i, int k)
{
  assert(k > i);
  for (int j = 0; j < (k-i+1)/2; ++j)
    std::swap(path[i+j], path[k-j]);
}

std::tuple<double, path_t> solve2opt (const distance_matrix_t& d)
{
  auto n (d.rows());
  // initial best
  path_t bestPath (n-1), currentPath(n-1);
  double bestCost;
  std::tie(bestCost, bestPath) = heuristic_nearest::solve(d);

  bool shorter;
  do {
    shorter = false;
    for (int i = 0; i < n-2; i++) {
      for (int k = i + 1; k < n-1; k++) {
        currentPath = bestPath;
        swap2opt(currentPath, i, k);
        double currentCost = evaluateCost(d, currentPath);
        if (currentCost < bestCost) {
          bestPath.swap(currentPath);
          bestCost = currentCost;
          shorter = true;
          break;
        }
      }
      if (shorter) break;
    }
  } while (shorter);

  return std::make_tuple(bestCost, bestPath);
}

/// If reversing tour[i:j] would make the tour shorter, then do it.
/// \return true if some elements where swapped to get a better path.
bool swap3opt(const distance_matrix_t& d, path_t& path, int i, int j, int k)
{
  // Given tour [...A-B...C-D...E-F...]
  int
    A = (i==0 ? 0 : path[i-1]), B = path[i],
    C = path[j-1], D = path[j],
    E = path[k-1], F = (k==path.size() ? 0 : path[k]);

  Eigen::Matrix<double, 8, 1> cases;
  cases <<
    d(A, B) + d(C, D) + d(E, F), // 0: initial
    d(A, B) + d(C, E) + d(D, F), // 1: reverse DE
    d(A, C) + d(B, D) + d(E, F), // 2: reverse BC
    d(A, C) + d(B, E) + d(D, F), // 3: reverse BC, DE
    d(A, D) + d(E, B) + d(C, F), // 4: exchange BC and DE
    d(A, D) + d(E, C) + d(B, F), // 5: 
    d(A, E) + d(D, B) + d(C, F), // 6: 
    d(A, E) + d(D, C) + d(B, F); // 7: 

  int minIndex;
  cases.minCoeff(&minIndex);
  double cost = evaluateCost(d, path);
  switch(minIndex) {
    case 0: return false;
    case 1: swap2opt(path, j, k-1); return true;
    case 2: swap2opt(path, i, j-1); return true;
    case 3: swap2opt(path, i, j-1); swap2opt(path, j, k-1); return true;
    case 4: {
              // Copy BC
              std::vector<int> BC(path.begin()+i, path.begin()+j);
              // Move DE after A
              auto B = std::copy(path.begin()+j, path.begin()+k, path.begin()+i);
              // Copy back BC
              std::copy(BC.begin(), BC.end(), B);
              return true;
            }
    case 5: {
              // Copy BC
              std::vector<int> BC(path.begin()+i, path.begin()+j);
              // Move DE after A
              auto B = std::copy(path.begin()+j, path.begin()+k, path.begin()+i);
              // Copy back CB
              std::copy(BC.rbegin(), BC.rend(), B);
              return true;
            }
    case 6: {
              // A  B  C  D        E  F
              // 0 {1, 2, 3, 4, 5, 6} 0
              // A  E        D  B  C  F
              //   {6, 5, 4, 3, 1, 2}
              // Copy BC and DE
              std::vector<int> BC(path.begin()+i, path.begin()+j);
              std::vector<int> DE(path.begin()+j, path.begin()+k);
              // Reverse and move DE after A
              auto B = std::copy(DE.rbegin(), DE.rend(), path.begin()+i);
              // Copy back BC
              std::copy(BC.begin(), BC.end(), B);
              return true;
            }
    case 7: {
              // A  B     C  D  E  F
              // 0 {1, 2, 3, 4, 5, 6} 0
              // A  E  D  C     B  F
              // 0 {5, 4, 3, 2, 1, 6} 0
              // Reverse BCDE
              swap2opt(path, i, k-1);
              return true;
            }
  };
}

/// Iterative improvement based on 3 exchange.
std::tuple<double, path_t> solve3opt (const distance_matrix_t& d)
{
  auto n (d.rows());
  path_t path;
  double cost;
  std::tie(cost, path) = heuristic_nearest::solve(d);

  bool improved;
  do {
    improved = false;
    for (int i = 0; i < n-1; ++i) { // index of B
      for (int j = i+2; j < n-1; ++j) { // index of D
        for (int k = j+2; k < n; ++k) { // index of F
          if (swap3opt(d, path, i, j, k)) {
            double ncost = evaluateCost(d, path);
            cost = ncost;
            improved = true;
            break;
          }
        }
        if (improved) break;
      }
      if (improved) break;
    }
  } while (improved);

  cost = evaluateCost(d, path);
  return std::make_tuple(cost, path);
}

}
}

// vim: foldmethod=syntax
