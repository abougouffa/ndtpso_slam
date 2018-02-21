#ifndef NDTPSO_BASE_H
#define NDTPSO_BASE_H

#include "ndtpso_slam/config.h"
#include "ndtpso_slam/ndtframe.h"
#include <eigen3/Eigen/Core>
#include <vector>

using std::vector;
using namespace Eigen;

Vector3d pso_optimization(Vector3d initial_guess, NdtFrame* const ref_frame, NdtFrame* const new_frame, unsigned int iters_num = 50, Array3d deviation = { 0, 0, 0 });
Vector3d glir_pso_optimization(Vector3d initial_guess, NdtFrame* const ref_frame, NdtFrame* const new_frame, unsigned int iters_num = 50);

// Spatial mapping T between two robot coordinate frames
// given point (the old frame origin), and trans (x, y and theta), return the new frame origin
inline Vector2d transform_point(Vector2d& point, Vector3d& trans)
{
    return Vector2d(point(0) * cos(trans[2]) - point(1) * sin(trans[2]) + trans[0],
        point(0) * sin(trans[2]) + point(1) * cos(trans[2]) + trans[1]);
};

inline vector<double> origin_at(Vector2d& point, double& cell_side)
{
    return { floor(point(0) / cell_side) * cell_side, floor(point(1) / cell_side) * cell_side };
};

// Convert an index to angle based on "step" and "minimum angle" (used in laser data)
#define INDEX_TO_ANGLE(idx, step, min_angle) (idx * step + min_angle)

// Convert from polar to cartesian coordinate, used in converting laser scans
// TODO: Add a L parameter to x (distance from robot gravity center to laser origin)
#define LASER_TO_POINT(r, theta) (Vector2d(r * cos(theta), r * sin(theta)))

#endif // NDTPSO_BASE_H
