#ifndef NDTFRAME_H
#define NDTFRAME_H
#include "ndtcell.h"
#include <eigen3/Eigen/Core>
#include <vector>

using namespace Eigen;
using namespace std;

#define LASER_IGNORE_EPSILON 0.05 // Ignore points around the origin with 5cm

// Convert an index to angle based on "step" and "minimum angle" (used in laser data)
#define INDEX_TO_ANGLE(idx, step, min_angle)     (idx * step + min_angle)

// Convert from polar to cartesian coordinate, used in converting laser scans
// TODO: Add a L parameter to x (distance from robot gravity center to laser origin)
#define LASER_TO_POINT(r, theta)                        (Vector2d(r * cos(theta), r * sin(theta)))


struct TransformParams
{
    double x, y, theta;
    TransformParams() {x = y = theta = 0.f;}
    TransformParams(double x_val, double y_val, double theta_val) {x = x_val; y = y_val; theta = theta_val;}
};

bool operator==(const TransformParams &left, const TransformParams &right);

// Spatial mapping T between two robot coordinate frames
// given point (the old frame origin), and trans (tx, ty and phi), return the new frame origin
inline Vector2d transform_point(Vector2d point, TransformParams trans) {
    return Vector2d(point(0) * cos(trans.theta) - point(1) * sin(trans.theta) + trans.x,
                    point(0) * sin(trans.theta) + point(1) * cos(trans.theta) + trans.y);
};

inline vector<double> origin_at(Vector2d point, double cell_side) {
    return {floor(point(0) / cell_side) * cell_side, floor(point(1) / cell_side) * cell_side};
};


using namespace std;

class NdtFrame
{
private:
    TransformParams _trans;
    double _cell_side;
public:
    uint16_t width, height, widthNumOfCells, heightNumOfCells;
    vector<NdtCell> cells;
    bool built;
    unsigned int numOfCells;
    NdtFrame(TransformParams trans, unsigned short width = 20, unsigned short height = 20, double cell_side = 1.0);
    void transform(TransformParams trans);
    void loadLaser(vector<double> laser_data, double min_angle = -M_PI_2, double max_angle = M_PI_2);
    void addPoint(Vector2d &point);
    void print();
    void build();
    inline int getCellIndex(Vector2d point);

};



double cost(TransformParams trans, NdtFrame * const ref_frame, NdtFrame * const new_frame);

#endif // NDTFRAME_H
