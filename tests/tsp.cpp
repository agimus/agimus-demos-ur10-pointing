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
  //run_benchs();
  return ::test::status;
}

// vim: foldmethod=syntax
