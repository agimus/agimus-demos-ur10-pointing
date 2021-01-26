#include "tsp.hpp"

#include <iostream>
#include <chrono>

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

#define CHECK_EQ(a,b) if (a != b) std::cerr << #a " (" << a << ") != " #b " (" << b << ")\n"
#define CHECK_LE(a,b) if (a > b) std::cerr << #a " (" << a << ") > " #b " (" << b << ")\n"

int main()
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
  int N = 8;
#else
  int N = 19;
#endif

  for (int n = 6; n < N; ++n) {
    d.resize(n,n);
    d.setRandom();
    d.array() += 1;
    d.diagonal().setZero();

    // Check neighbor matrix
    tsp::ordered_neighbors_t nn (tsp::neighborMatrix(d));
    CHECK_EQ(nn.rows(), n-1);
    CHECK_EQ(nn.cols(), n);
    for (int i = 0; i < n; ++i) {
      for (int k = 0; k < n-2; ++k) {
        CHECK_LE(d(nn(k,i),i), d(nn(k+1,i),i));
      }
    }

    compare(d);
  }
}

// vim: foldmethod=syntax
