#include "ndtpso_slam/ndtcell.h"
#include <cstdio>
#include <eigen3/Eigen/Eigen>

NDTCell::NDTCell(bool calculate_params) {
  if (calculate_params) {
    for (unsigned int i = 0; i < NDT_WINDOW_SIZE; ++i) {
      this->s_partial_sums[i] = Vector2d::Zero();
      this->s_partial_counts[i] = 0;
      this->s_partial_covars[i] = Matrix2d::Zero();
    }

    this->s_global_sum = Vector2d::Zero();
    this->s_global_covar_sum = Matrix2d::Zero();
    this->s_global_count = 0;
  }

  this->s_current_partial_sum = Vector2d::Zero();
}

void NDTCell::addPoint(const Vector2d &point) {
  if (0 == this->s_current_count) {
    // For the first call after building the cell (so the first call after the
    // previous iteration) we reset all the points of the corresponding
    // iteration in the points buffer
    this->points[this->s_current_window_id] = vector<Vector2d>();
  }

  this->s_current_count++;
  this->s_current_partial_sum += point;
  this->points[this->s_current_window_id].push_back(std::move(point));
  this->created = true;
  this->built = false;
}

bool NDTCell::build() {
  // Managing the circular buffer "the window" in a constant time
  WINDOW_ADD(this->s_global_sum, this->s_current_partial_sum,
             this->s_partial_sums, this->s_current_window_id);
  WINDOW_ADD(this->s_global_count, this->s_current_count,
             this->s_partial_counts, this->s_current_window_id);

  if (this->s_global_count > 2) {
    this->mean = this->s_global_sum / this->s_global_count;

    Matrix2d cov = Matrix2d::Zero();
    Vector2d distance_from_mean;

    for (auto &pt : this->points[this->s_current_window_id]) {
      distance_from_mean = pt - this->mean;
      cov += (distance_from_mean * distance_from_mean.transpose());
    }

    WINDOW_ADD(this->s_global_covar_sum, cov, this->s_partial_covars,
               this->s_current_window_id);

    this->s_calc_covar_inverse();
    this->built = true;
  }

  if (this->s_current_count > NDT_MAX_POINTS_PER_CELL) {
    WINDOW_INC_ID(this->s_current_window_id);
    this->s_current_count = 0;
    this->s_current_partial_sum = Vector2d::Zero();
  }

  return this->built;
}

double NDTCell::normalDistribution(const Vector2d &point) {
  if (this->built) {
    Vector2d diff = point - this->mean;
    return exp(
        -static_cast<double>((diff.transpose() * this->s_inv_covar) * diff) /
        2.) /* + .5*/;
  }
  return 0;
}

void NDTCell::reset() {
  this->s_current_partial_sum = Vector2d::Zero();
  this->s_global_sum = Vector2d::Zero();
  this->s_current_count = 0;
  this->s_global_count = 0;
  this->s_global_covar_sum = Matrix2d::Zero();
  this->s_current_window_id = 0;

  for (auto &point : this->points) {
    point.clear();
  }
}

void NDTCell::s_calc_covar_inverse() {
  Matrix2d covar = this->s_global_covar_sum / this->s_global_count;

  EigenSolver<Matrix2d> eigenval_solver(covar);
  Vector2d eigenvals = eigenval_solver.pseudoEigenvalueMatrix().diagonal();
  double large_val, small_val;

  large_val = eigenvals[eigenvals[0] > eigenvals[1] ? 0 : 1];
  small_val = eigenvals[eigenvals[0] < eigenvals[1] ? 0 : 1];

  if (small_val < .001 * large_val)
    large_val = .001 * large_val *
                large_val; // From now, the large_val will hold the determinant
  else
    large_val = covar.determinant();

  this->s_inv_covar << covar(1, 1) / large_val, -covar(0, 1) / large_val,
      -covar(1, 0) / large_val, covar(0, 0) / large_val;
}
