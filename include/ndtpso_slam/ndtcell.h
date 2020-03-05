#ifndef NDTCELL_H
#define NDTCELL_H

#include "config.h"
#include <eigen3/Eigen/Core>
#include <vector>

using namespace Eigen;
using std::vector;

class NDTCell {
private:
    Vector2d _frame_sums[NDT_WINDOW_SIZE], _frame_sum, _global_sum;
    Matrix2d _frame_covars[NDT_WINDOW_SIZE], _global_covar_sum, _inv_covar;
    int _points_nums[NDT_WINDOW_SIZE], _points_num, _global_points_num;
    inline void _calc_covar_inverse();

public:
    unsigned int frame_count;
    std::vector<Vector2d> points;
    Vector2d mean;
    NDTCell(std::vector<Vector2d> points = {});
    void addPoint(Vector2d point);
    bool built;
    bool created;
    bool build();
    double normalDistribution(Vector2d point);
    void reset();
};

#endif // NDTCELL_H
