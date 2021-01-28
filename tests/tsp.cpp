#include "tsp.hpp"

#include <iterator>
#include <iostream>
#include <chrono>

namespace test {
  int status = 0;
}

#define _STRINGIFY(x) #x
#define STRINGIFY(x) _STRINGIFY(x)
#define CHECK(check, stream) if (!(check)) { ::test::status = 1; std::cerr << __FILE__ ":" STRINGIFY(__LINE__) ": Failed check: " << stream; }
#define _CHECK_COMPARISION(a,op,b) CHECK(a op b, #a " (" << (a) << ") " #op " " #b " (" << (b) << ")\n")
#define CHECK_EQ(a,b) _CHECK_COMPARISION(a, ==, b)
#define CHECK_LE(a,b) _CHECK_COMPARISION(a, <=, b)
#define CHECK_LT(a,b) _CHECK_COMPARISION(a, < , b)

std::ostream& operator<<(std::ostream& os, const tsp::path_t& p)
{
  os << "[ ";
  std::copy(p.begin(), p.end(), std::ostream_iterator<tsp::path_t::value_type>(os, ", "));
  return os << ']';
}

void test_randomMatrix()
{
  int n = 5;
  tsp::distance_matrix_t d = tsp::randomDistanceMatrix(n);
  for (int i = 0; i < n; ++i) {
    CHECK_EQ(d(i,i), 0);
    for (int j = i+1; j < n; ++j)
      CHECK_EQ(d(i,j), d(j,i));
  }
}

void test_neighborMatrix()
{
  int n = 30;
  for (int i = 0; i < 5; ++i) {
    auto d = tsp::randomDistanceMatrix(n);

    // Check neighbor matrix
    tsp::ordered_neighbors_t nn (tsp::neighborMatrix(d));
    CHECK_EQ(nn.rows(), n-1);
    CHECK_EQ(nn.cols(), n);
    for (int i = 0; i < n; ++i) {
      for (int k = 0; k < n-2; ++k) {
        CHECK_LE(d(nn(k,i),i), d(nn(k+1,i),i));
      }
    }
  }
}

void test_swap2opt()
{
  using tsp::path_t;
  using tsp::approximative_kopt::swap2opt;
  {
    path_t path { 0, 1, 2, 3, 4 },
           expp { 3, 2, 1, 0, 4 };
    swap2opt(path, 0, 3);
    CHECK_EQ(path, expp);
  }
  {
    path_t path { 0, 1, 2, 3, 4 },
           expp { 0, 1, 3, 2, 4 };
    swap2opt(path, 2, 3);
    CHECK_EQ(path, expp);
  }
}

void test_swap3opt()
{
  using tsp::path_t;
  using tsp::approximative_kopt::swap3opt;

  int n = 7;
  for (int i = 0; i < 5; ++i) {
    tsp::distance_matrix_t d = tsp::randomDistanceMatrix(n);

    path_t pathBefore { 1, 2, 3, 4, 5, 6 }, pathAfter (pathBefore);
    double costBefore = tsp::evaluateCost(d, pathBefore);

    CHECK_EQ(pathBefore.size(), n-1);
    for (int i = 0; i < n-1; ++i) { // index of B
      int A = (i==0 ? 0 : pathBefore[i-1]), B = pathBefore[i];
      for (int j = i+2; j < n-1; ++j) { // index of D
        int C = pathBefore[j-1], D = pathBefore[j];
        for (int k = j+2; k < n; ++k) { // index of F
          int E = pathBefore[k-1], F = (k==pathBefore.size() ? 0 : pathBefore[k]);

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

          bool res = swap3opt(d, pathAfter, i, j, k);

          CHECK_EQ(res, minIndex>0);
          
          double costAfter = tsp::evaluateCost(d, pathAfter);
          if (res) {
            CHECK_LT(costAfter, costBefore);
            pathAfter = pathBefore;
          } else
            CHECK_EQ(pathBefore, pathAfter);
        }
      }
    }
  }
}

void print(tsp::path_t p)
{
  std::cout << "[ ";
  for (auto k : p)
    std::cout << k << ", ";
  std::cout << "] ";
}

void compare(const tsp::distance_matrix_t& d)
{
  typedef std::chrono::high_resolution_clock clock;
  typedef std::chrono::duration<double, std::micro> duration;

  auto measure = [&d](auto func, const char* header) {
    auto t1 (clock::now());
    auto res = func(d);
    auto t2 (clock::now());

    print(std::get<1>(res));
    std::cout << header << ": cost(" << std::get<0>(res) << ") duration(" << duration(t2-t1).count() << ")\n";
  };

  //std::cout << d << '\n';
  std::cout << "=================================\n";
  measure(tsp::heuristic_nearest::solve, "HN ");
  //std::cout << "---------------------------------\n";
  measure(tsp::approximative_kopt::solve2opt, "A2O");
  //std::cout << "---------------------------------\n";
  measure(tsp::approximative_kopt::solve3opt, "A3O");
  //std::cout << "---------------------------------\n";
  measure(tsp::dynamic_programming::solveWithHeuristic, "DPH");
  //std::cout << "---------------------------------\n";
  measure(tsp::dynamic_programming::solve, "DP ");
  //std::cout << "---------------------------------\n";
  if (d.rows() < 12)
    measure(tsp::brute_force::solve, "BF ");
  else std::cout << "BF: skipped\n";
  //std::cout << "---------------------------------\n";
}

void run_tests()
{
  test_randomMatrix();
  test_neighborMatrix();
  test_swap2opt();
  test_swap3opt();
}

void run_benchs()
{
  Eigen::MatrixXd d (4,4);

  /*
  d << 0, 1, 2, 3,
       0, 0, 4, 5,
       0, 0, 0, 6,
       0, 0, 0, 0;
  d.triangularView<Eigen::StrictlyLower>() = d.transpose().triangularView<Eigen::StrictlyLower>();
  compare(d);
  */

#ifndef NDEBUG
  int N = 9;
#else
  int N = 20;
#endif

  for (int n = 7; n < N; ++n) {
    d = tsp::randomDistanceMatrix(n);
    compare(d);
  }
}

int main()
{
  run_tests();
  run_benchs();
  return ::test::status;
}

// vim: foldmethod=syntax
