#include "ndtpso_slam/ndtcell.h"
#include <eigen3/Eigen/Eigen>
#include <stdio.h>

NDTCell::NDTCell()
    : _current_count(0)
    , _current_window_id(0)
    , built(false)
    , created(false)
{
    for (unsigned int i = 0; i < NDT_WINDOW_SIZE; ++i) {
        this->_partial_sums[i] << 0, 0;
        this->_partial_counts[i] = 0;
        this->_partial_covars[i] << 0, 0,
            0, 0;
    }

    this->_current_partial_sum << 0, 0;
    this->_global_sum << 0, 0;
    this->_current_count = 0;
}

void NDTCell::addPoint(Vector2d point)
{
    this->_current_count++;
    this->_current_partial_sum += point;
    this->points.push_back(point);
    this->created = true;
}

bool NDTCell::build()
{
    this->_global_sum = this->_global_sum + this->_current_partial_sum - this->_partial_sums[this->_current_window_id];
    this->_partial_sums[this->_current_window_id] = this->_current_partial_sum;

    this->_global_count = this->_global_count + this->_current_count - this->_partial_counts[this->_current_window_id];
    this->_partial_counts[this->_current_window_id] = this->_current_count;

    if (this->_global_count > 2) {
        this->mean = this->_global_sum / this->_global_count;
        this->_calc_covar_inverse();
        this->built = true;
    }

    this->_current_window_id = (this->_current_window_id + 1) % NDT_WINDOW_SIZE;

    return this->built;
}

double NDTCell::normalDistribution(Vector2d point)
{
    if (this->built) {
        Vector2d diff = point - this->mean;
        return exp(-static_cast<double>((diff.transpose() * this->_inv_covar) * diff) / 2.) + .5;
    } else {
        return 0;
    }
}

void NDTCell::reset()
{
    this->_current_partial_sum << 0, 0;
    this->_current_count = 0;
    this->points.clear();
}

void NDTCell::_calc_covar_inverse()
{
    Matrix2d cov;
    cov << .0, .0,
        .0, .0;

    Vector2d tmp_pt;

    for (uint16_t i = 0; i < this->_current_count; ++i) {
        tmp_pt = this->points[i] - this->mean;
        cov += (tmp_pt * tmp_pt.transpose());
    }

    this->_global_covar_sum = (this->_global_covar_sum + cov) - this->_partial_covars[this->_current_window_id];
    this->_partial_covars[this->_current_window_id] = cov;

    Matrix2d covar = this->_global_covar_sum / this->_global_count;

    EigenSolver<Matrix2d> eigenval_solver(covar);
    Vector2d eigenvals = eigenval_solver.pseudoEigenvalueMatrix().diagonal();
    double large_val, small_val;

    large_val = eigenvals[eigenvals[0] > eigenvals[1] ? 0 : 1];
    small_val = eigenvals[eigenvals[0] < eigenvals[1] ? 0 : 1];

    if (small_val < .001 * large_val)
        large_val = .001 * large_val * large_val; // From now, the large_val will hold the determinant
    else
        large_val = covar.determinant();

    this->_inv_covar << covar(1, 1) / large_val, -covar(0, 1) / large_val,
        -covar(1, 0) / large_val, covar(0, 0) / large_val;
}
