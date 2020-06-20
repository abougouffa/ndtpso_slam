#ifndef NDTPSO_BASE_H
#define NDTPSO_BASE_H

#include "ndtpso_slam/config.h"
#include "ndtpso_slam/ndtframe.h"
#include <eigen3/Eigen/Core>
#include <vector>

using Eigen::Array2d;
using Eigen::Array3d;
using Eigen::Matrix2d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using std::vector;

Vector3d pso_optimization(
    Vector3d initial_guess,
    NDTFrame* ref_frame,
    const NDTFrame* const new_frame,
    const Array3d& deviation = { 0, 0, 0 },
    const PSOConfig& pso_conf = PSOConfig());

Vector3d glir_pso_optimization(
    Vector3d initial_guess,
    NDTFrame* ref_frame,
    NDTFrame* new_frame,
    unsigned int iters_num = 50,
    const Array3d& deviation = { 0, 0, 0 });

// Spatial mapping T between two robot coordinate frames
// given point (the old frame origin), and trans (x, y and theta), return the new frame origin
inline Vector2d transform_point(const Vector2d& point, const Vector3d& trans)
{
    return { point.x() * cos(trans.z()) - point.y() * sin(trans.z()) + trans.x(),
        point.x() * sin(trans.z()) + point.y() * cos(trans.z()) + trans.y() };
}

inline vector<double> origin_at(Vector2d& point, double& cell_side)
{
    return { floor(point.x() / cell_side) * cell_side, floor(point.y() / cell_side) * cell_side };
}

// Convert an index to angle based on "step" and "minimum angle" (used in laser data)
inline float index_to_angle(unsigned int idx, float step, float min_angle)
{
    return idx * step + min_angle;
}

// Convert from polar to cartesian coordinate, used in converting laser scans
inline Vector2d laser_to_point(float r, float theta)
{
    return { double(r) * cos(double(theta)), double(r) * sin(double(theta)) };
}

double cost_function(Vector3d trans, NDTFrame* const ref_frame, const NDTFrame* const new_frame);

#endif // NDTPSO_BASE_H
