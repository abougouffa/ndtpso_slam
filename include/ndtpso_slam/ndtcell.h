#ifndef NDTCELL_H
#define NDTCELL_H

#include "config.h"
#include <eigen3/Eigen/Core>
#include <vector>

using namespace Eigen;
using std::vector;

class NDTCell {
private:
    Vector2d _partial_sums[NDT_WINDOW_SIZE], _current_partial_sum, _global_sum;
    Matrix2d _partial_covars[NDT_WINDOW_SIZE], _global_covar_sum, _inv_covar;
    int _partial_counts[NDT_WINDOW_SIZE], _current_count, _global_count;
    size_t _current_window_id;
    inline void _calc_covar_inverse();

public:
    std::vector<Vector2d> points;
    Vector2d mean;
    NDTCell();
    void addPoint(Vector2d point);
    bool built;
    bool created;
    bool build();
    double normalDistribution(Vector2d point);
    void reset();
};

#endif // NDTCELL_H
