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

// At most 32 cities
typedef uint8_t city_index_type;
typedef uint32_t city_set_t;
typedef std::vector<path_t> cache_t;

constexpr city_set_t all_cities (~city_set_t(0));

struct DP {
  const distance_matrix_t& d;
  const ordered_neighbors_t neighbors;
  double dmin;

  double costUpperBound = std::numeric_limits<double>::infinity();

  bool useCachedCosts = true, allowPruning = false;
  typedef struct { double cost; city_index_type prevCity, ineighbor; TSP_DP_IN_DEBUG(int nvisits;) } cache_type;
  std::map<uint64_t, cache_type> cachedCosts;

#ifdef TSP_DP_DEBUG
  long int ncalls = 0, nroots = 0, nupdates = 0, nprunes = 0;
#endif

  DP(const distance_matrix_t& _d)
    : d(_d), neighbors(neighborMatrix(d))
  {
    dmin = std::numeric_limits<double>::infinity();
    for (int i = 1; i < d.rows(); ++i) {
      dmin = std::min({
          d.row(i).head(i).minCoeff(),
          d.row(i-1).tail(d.rows()-i).minCoeff(),
          dmin});
    }
  }

  // Compute the distance to go from city 0, visiting all cities in N and finishing at i \notin N
  // \param dmin the minimal distance between two different cities.
  // \param i final city.
  // \param N an integer with one bit per city, indicating whether it is in the set.
  // \param nN the number of cities in the set (for optimization only, redundant with N).
  // \param depth should be d.rows() - nN (-1?)
  // \param costToCome
  double dist (int i, city_set_t N, int nN, int depth, double costToCome);

  static constexpr uint64_t make_key(city_set_t N, city_index_type i) { return uint64_t(N) | (uint64_t(i) << 32); }

  path_t solution() const {
    // All cities except 0
    city_index_type n(d.cols());
    city_set_t N = (all_cities >> (64-n)) & (all_cities << 1);

    path_t sol (n-1);
    city_index_type last = 0;
    for (city_index_type k = 0; k < n-1; ++k) {
      auto key (make_key(N, last));
      decltype(cachedCosts.cbegin()) _cached = cachedCosts.find(key);
      if (_cached == cachedCosts.end())
        throw std::runtime_error("cached cost map does not contain a valid solution.");
      last = _cached->second.prevCity;
      sol[n-2-k] = (int)last;
      N = N & ~(1 << last);
    }
    return sol;
  }
};

double DP::dist (int i, city_set_t N, int nN, int depth, double costToCome)
{
  TSP_DP_IN_DEBUG(++ncalls;)
  if (nN == 0) {
    TSP_DP_IN_DEBUG(++nroots;)
    if (costToCome + d(0,i) < costUpperBound) {
      costUpperBound = costToCome + d(0,i);
      TSP_DP_IN_DEBUG(nupdates++;)
    }
    return d(0, i);
  }
  //if (costToCome + (nN+1)*dmin > costUpperBound) // cannot do better
    //return std::numeric_limits<double>::infinity();
  // Make the key
  uint64_t key = make_key(N, i);
  decltype(cachedCosts.emplace(key, cache_type())) cachedCost;
  cachedCost = cachedCosts.emplace(key, cache_type{ std::numeric_limits<double>::infinity(), 0, 0 });
  cache_type& cached = cachedCost.first->second;
  TSP_DP_IN_DEBUG(cached.nvisits++;)
  bool newCall = cachedCost.second;
  if (useCachedCosts && cached.ineighbor == neighbors.rows()) // return calculated cost
    return cached.cost;

  // Store the costs in the form (nj, dist(nj, N))
  city_set_t NN;
  int k = 0;
  assert(d.cols() == neighbors.rows()+1);
  // Recompute upper bound because costUpperBound may be smaller
  if (costToCome + cached.cost < costUpperBound)
    costUpperBound = costToCome + cached.cost;
  for (;cached.ineighbor < neighbors.rows(); ++cached.ineighbor) { // Loop on the neighbors by increasing order of d(k, i)
    int j = neighbors(cached.ineighbor, i);
    if (!(N & (1 << j))) continue;
    assert(j != i);
    // nN * dmin is the minimum cost to visit all cities in NN (nN-1 cities) plus
    // a segment to reach j.
    // So the test below stops recursion if it is not possible to generate a path
    // with lower cost than costUpperBound.
    if (allowPruning && nN * dmin + d(j,i) + costToCome >= costUpperBound) {
      TSP_DP_IN_DEBUG(nprunes++;)
      break;
    }
    NN = N & ~(1 << j);
    double cost = dist(j, NN, nN-1, depth+1, costToCome + d(j,i)) + d(j,i);
    if (cost < cached.cost) {
      cached.cost = cost;
      cached.prevCity = j;
    }
    if (++k == nN) {
      cached.ineighbor = neighbors.rows();
      break;
    }
  }
  return cached.cost;
}

std::tuple<double, path_t> solveWithBound (const distance_matrix_t& d,
    double costUpperBound,
    bool allowPruning)
{
  if (d.cols() != d.rows())
    throw std::invalid_argument("expect a square distance matrix");
  auto n = d.cols();
  if (n >= 32)
    throw std::invalid_argument("at the moment, cannot solve instance with more than 31 cities.");

  city_set_t N = (all_cities >> (64-n)) & (all_cities << 1);

  DP dp (d);
  dp.costUpperBound = costUpperBound;
  dp.useCachedCosts = true;
  dp.allowPruning = allowPruning;

  double cost = dp.dist(0, N, n-1, 0, 0);
#ifdef TSP_DP_DEBUG
  /*
  std::cout << n << ' ' << dp.nupdates << '/' << dp.nroots << '/' << dp.ncalls << std::endl;
  long int total = 0;
  for (auto pair : dp.cachedCosts) {
    int finalcity = pair.first >> 32;
    uint32_t N (pair.first);
    std::cout << std::setfill(' ') << std::setw(6) << std::hex << N << ' '
      << std::setw(5) << std::dec << finalcity << ' ' << pair.second.nvisits << '\n';
    total += pair.second.nvisits;
  }
  std::cout << dp.cachedCosts.size() << '/' << total << " .....\n";
  */
#endif
  return std::make_tuple(cost, dp.solution());
}

std::tuple<double, path_t> solveWithHeuristic (const distance_matrix_t& d,
    bool allowPruning)
{
  double costUpperBound;
  path_t path;
  //std::tie(costUpperBound, path) = heuristic_nearest::solve(d);
  std::tie(costUpperBound, path) = approximative_kopt::solve3opt(d);
  return solveWithBound(d, 1.00001 * costUpperBound);
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

std::tuple<double, path_t> solve2opt (const distance_matrix_t& d,
    path_t initialGuess)
{
  auto n (d.rows());
  // initial best
  path_t bestPath (n-1), currentPath(n-1);
  double bestCost;
  if (initialGuess.size() == 0)
    std::tie(bestCost, bestPath) = heuristic_nearest::solve(d);
  else if (initialGuess.size() == n-1) {
    bestPath = initialGuess;
    bestCost = evaluateCost(d, bestPath);
  } else
    throw std::invalid_argument("invalid initial guess.");

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
  if (minIndex > 0 && cases[0] - cases[minIndex] < 1e-12)
    minIndex = 0;
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
std::tuple<double, path_t> solve3opt (const distance_matrix_t& d,
    path_t initialGuess)
{
  auto n (d.rows());
  path_t path;
  double cost;
  if (initialGuess.size() == 0)
    std::tie(cost, path) = heuristic_nearest::solve(d);
  else if (initialGuess.size() == n-1) {
    path = initialGuess;
    cost = evaluateCost(d, path);
  } else
    throw std::invalid_argument("invalid initial guess.");

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
