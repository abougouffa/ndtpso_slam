#include "ndtpso_slam/ndtcell.h"
#include <eigen3/Eigen/Eigen>
#include <stdio.h>

using namespace Eigen;
using namespace std;


NdtCell::NdtCell(vector<Vector2d> points) :
    points(points)
{
    this->isBuilt = false;
    this->created = false;
}



void NdtCell::print(int index)
{
    for (unsigned int i = 0; i < this->points.size(); ++i) {
        printf("%d, %f, %f\n", index, this->points[i][0], this->points[i][1]);
    }
}

void NdtCell::addPoint(Vector2d point)
{
    this->points.push_back(point);
    this->created = true;
}

bool NdtCell::build()
{
    if (this->points.size() > 2) {
        this->_calc_mean();
        this->_calc_covar();
        this->_calc_covar_inverse();
        this->isBuilt = true;
    }

    return this->isBuilt;
}

double NdtCell::normalDistribution(Vector2d point)
{
    if (this->isBuilt) {
        Vector2d diff = point - this->mean;
//        std::cout << ((pt_diff.transpose() * this->_inv_covar) * pt_diff) / 2. << std::endl;
        return exp(-static_cast<double>((((diff.transpose() * this->_inv_covar) * diff))) / 2.);
    } else {
        return -1;
    }
}

void NdtCell::_calc_mean()
{
    this->mean = Vector2d(0, 0);
    uint16_t num_pts = static_cast<uint16_t>(this->points.size());

    for (uint16_t i = 0; i < num_pts; ++i) {
        this->mean += this->points[i];
    }

    this->mean /= num_pts;
}

void NdtCell::_calc_covar()
{
    this->_covar <<     .0, .0,
                        .0, .0;

    uint16_t num_pts = static_cast<uint16_t>(this->points.size());
    Vector2d tmp_pt;

    for (uint16_t i = 0; i < num_pts; ++i) {
        tmp_pt = this->points[i] - this->mean;
        this->_covar += (tmp_pt * tmp_pt.transpose()) / num_pts;
    }
}

void NdtCell::_calc_covar_inverse()
{
    EigenSolver<Matrix2d> eigenval_solver(this->_covar);
    Vector2d eigenvals = eigenval_solver.pseudoEigenvalueMatrix().diagonal();
    double large_val, small_val;

    large_val = eigenvals[eigenvals[0] > eigenvals[1]? 0: 1];
    small_val = eigenvals[eigenvals[0] < eigenvals[1]? 0: 1];

    if (small_val < .001 * large_val)
        large_val = .001 * large_val * large_val; // From now, the large_val hold the new determinant
    else
        large_val = this->_covar.determinant();

    this->_inv_covar <<      this->_covar(1, 1) / large_val, -this->_covar(0, 1) / large_val,
                            -this->_covar(1, 0) / large_val,  this->_covar(0, 0) / large_val;
//    std::cout << this->_inv_covar << std::endl;
}


