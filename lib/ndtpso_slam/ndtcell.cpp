#include "ndtpso_slam/ndtcell.h"
#include <eigen3/Eigen/Eigen>
#include <stdio.h>

NDTCell::NDTCell(vector<Vector2d> points)
    : _points_num(0)
    , frame_id(0)
    , points(points)
    , built(false)
    , created(false)
{
    for (unsigned int i = 0; i < NDT_WINDOW_SIZE; ++i) {
        this->_frame_sums[i] << 0, 0;
        this->_points_nums[i] = 0;
        this->_frame_covars[i] << 0, 0,
            0, 0;
    }

    this->_frame_sum << 0, 0;
    this->_global_sum << 0, 0;
    this->_points_num = 0;
}

void NDTCell::print(int index)
{
    for (unsigned int i = 0; i < this->points.size(); ++i)
        printf("@%d, (%f, %f)\n", index, this->points[i][0], this->points[i][1]);
}

void NDTCell::addPoint(Vector2d point)
{
    this->_points_num++;
    this->_frame_sum += point;
    this->points.push_back(point);
    this->created = true;
}

bool NDTCell::build()
{
    this->_global_sum = this->_global_sum + this->_frame_sum - this->_frame_sums[this->frame_id];
    this->_frame_sums[this->frame_id] = this->_frame_sum;

    this->_global_points_num = this->_global_points_num + this->_points_num - this->_points_nums[this->frame_id];
    this->_points_nums[this->frame_id] = this->_points_num;

    if (this->points.size() > 2) {
        this->mean = this->_global_sum / this->_global_points_num;
        this->_calc_covar_inverse();
        this->built = true;
    }

    this->frame_id = (this->frame_id + 1) % NDT_WINDOW_SIZE;

    return this->built;
}

double NDTCell::normalDistribution(Vector2d point)
{
    if (this->built) {
        Vector2d diff = point - this->mean;
        return exp(-static_cast<double>((diff.transpose() * this->_inv_covar) * diff) / 2.) + .5;
    } else {
        return -.5;
    }
}

void NDTCell::resetPoints()
{
    this->_frame_sum << 0, 0;
    this->_points_num = 0;
    this->points.clear();
}

void NDTCell::_calc_covar_inverse()
{
    Matrix2d cov;
    cov << .0, .0,
        .0, .0;

    Vector2d tmp_pt;

    for (uint16_t i = 0; i < this->_points_num; ++i) {
        tmp_pt = this->points[i] - this->mean;
        cov += (tmp_pt * tmp_pt.transpose());
    }

    this->_global_covar_sum = (this->_global_covar_sum + cov) - this->_frame_covars[this->frame_id];
    this->_frame_covars[this->frame_id] = cov;

    Matrix2d covar = this->_global_covar_sum / this->_global_points_num;

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
