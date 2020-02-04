#ifndef NDTFRAME_H
#define NDTFRAME_H

#include "ndtpso_slam/ndtcell.h"
#include <eigen3/Eigen/Core>
#include <vector>

using namespace Eigen;
using std::vector;

class NDTFrame {
private:
    Vector3d _trans;
    vector<Vector3d> _poses, _odoms;
    vector<double> _timestamps;
    bool _positive_only;
    double _x_min, _x_max, _y_min, _y_max;
    NdtPsoConfig _config;

public:
    uint16_t width, height, widthNumOfCells, heightNumOfCells;
    vector<NDTCell> cells;
    bool built;
    unsigned int numOfCells;
    double cell_side;
    NDTFrame(Vector3d trans, unsigned short width = 20, unsigned short height = 20, double cell_side = 1.0, bool positive_only = false);
    void transform(Vector3d trans);
    void loadLaser(const vector<float>& laser_data, const float& min_angle, const float& angle_increment, const float& max_range);
    void update(Vector3d trans, NDTFrame* const new_frame);
    void addPoint(Vector2d& point);
    inline void setTrans(Vector3d trans) { this->_trans = trans; }
#if defined(DEBUG) && DEBUG
    void print();
#endif
    void build();
    inline int getCellIndex(Vector2d point);
    Vector3d align(Vector3d initial_guess, NDTFrame* const new_frame);
    void dumpMap(const char* const filename, bool save_poses = true, bool save_points = true, bool save_image = true, short density = 50);
    void addPose(double timestamp, Vector3d pose, Vector3d odom = Vector3d::Zero());
    void resetCells();
};

extern double cost_function(Vector3d trans, NDTFrame* const ref_frame, NDTFrame* const new_frame);

#endif // NDTFRAME_H
