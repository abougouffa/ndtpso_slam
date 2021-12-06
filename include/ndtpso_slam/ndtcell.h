#ifndef NDTCELL_H
#define NDTCELL_H

#include "config.h"
#include <eigen3/Eigen/Core>
#include <vector>

// 1. The global sum
//        (plus) the currnet partial sum
//        (minus) the partial sum of the item we will replace with the current
//        partial sum
// 2. We replace the item
#define WINDOW_ADD(global, partial, partials, idx)                             \
  (global = (global + partial - partials[idx]));                               \
  (partials[idx] = partial)

// Circular increment of index (idx)
#define WINDOW_INC_ID(idx) (idx = ((idx + 1) % NDT_WINDOW_SIZE))
template <class T> struct CircularMean {
private:
  T current_partial_sum_, global_sum_;
  size_t win_index_{0}, used_window_{0};

  void addToWindow_() {
    // 1. The global sum
    //        (plus) the current term (partial sum)
    //        (minus) the partial sum of the item we will replace with the current
    //        partial sum
    // 2. We replace the item
    global_sum_ = global_sum_ + current_partial_sum_ - partial_terms[win_index_];
    partial_terms[win_index_] = current_partial_sum_;
  }

public:
    T partial_terms[NDT_WINDOW_SIZE];
    CircularMean() : current_partial_sum_(T{}), global_sum_(T{}) {
    for (auto& term : partial_terms) {
      term = T{};
    }
  }

  void increment() {
    // Circular increment of index (win_index_)
    win_index_ = (win_index_ + 1) % NDT_WINDOW_SIZE;

    if (used_window_ < NDT_WINDOW_SIZE) {
      used_window_ += 1;
    }

    addToWindow_(current_partial_sum_);
    current_partial_sum_ = T{};
  }

  T getMean() { return global_sum_ / used_window_; }
  T getSum() { return global_sum_; }
  T getPartialSum() { return current_partial_sum_; }
  size_t getUsedWindow() { return used_window_; }
};

using namespace Eigen;
using std::vector;

class NDTCell {
private:
  Vector2d s_partial_sums[NDT_WINDOW_SIZE], s_current_partial_sum, s_global_sum;
  Matrix2d s_partial_covars[NDT_WINDOW_SIZE], s_global_covar_sum, s_inv_covar;
  int s_partial_counts[NDT_WINDOW_SIZE], s_current_count{0}, s_global_count{0};
  size_t s_current_window_id{0};
  inline void s_calc_covar_inverse();

public:
  std::vector<Vector2d> points_vector[NDT_WINDOW_SIZE];
  Vector2d mean;
  NDTCell(bool init_cell_window = true);
  void addPoint(const Vector2d& point);
  bool built{false};
  bool created{false};
  bool build();
  double normalDistribution(const Vector2d& point);
  void reset();
};

#endif // NDTCELL_H
