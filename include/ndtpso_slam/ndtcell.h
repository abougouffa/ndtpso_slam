#ifndef NDTCELL_H
#define NDTCELL_H

#include <eigen3/Eigen/Core>
#include <vector>

using namespace Eigen;
using std::vector;

class NdtCell {
private:
    Matrix2d _covar, _inv_covar;
    inline void _calc_mean();
    inline void _calc_covar();
    inline void _calc_covar_inverse();

public:
    //    vector<double> origin;
    std::vector<Vector2d> points;
    Vector2d mean;
    NdtCell(std::vector<Vector2d> points = {});
    void print(int index);
    void addPoint(Vector2d point);
    bool isBuilt;
    bool created;
    bool build();
    double normalDistribution(Vector2d point);
};

#endif // NDTCELL_H
