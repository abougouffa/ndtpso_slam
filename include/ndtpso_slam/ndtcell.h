#ifndef NDTCELL_H
#define NDTCELL_H

#include "config.h"
#include <eigen3/Eigen/Core>
#include <vector>

using namespace Eigen;
using std::vector;

class NDTCell {
private:
    Vector2d s_partial_sums[NDT_WINDOW_SIZE], s_current_partial_sum, s_global_sum;
    Matrix2d s_partial_covars[NDT_WINDOW_SIZE], s_global_covar_sum, s_inv_covar;
    int s_partial_counts[NDT_WINDOW_SIZE], s_current_count{ 0 }, s_global_count{ 0 };
    size_t s_current_window_id{ 0 };
    inline void s_calc_covar_inverse();

public:
    std::vector<Vector2d> points[NDT_WINDOW_SIZE];
    Vector2d mean;
    NDTCell(bool calculate_params = true);
    void addPoint(const Vector2d& point);
    bool built{ false };
    bool created{ false };
    bool build();
    double normalDistribution(const Vector2d& point);
    void reset();
};

#endif // NDTCELL_H
