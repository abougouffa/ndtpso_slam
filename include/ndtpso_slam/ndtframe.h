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
    vector<Vector3d> _poses;
    bool _positive_only;
    float _x_min, _x_max, _y_min, _y_max;

public:
    uint16_t width, height, widthNumOfCells, heightNumOfCells;
    vector<NDTCell> cells;
    bool built;
    unsigned int numOfCells;
    double cell_side;
    void transform(Vector3d trans);
    void loadLaser(vector<float> laser_data, float min_angle, float max_angle, float angle_increment);
    NDTFrame(Vector3d trans, unsigned short width = 20, unsigned short height = 20, double cell_side = 1.0, double positive_only = false);
    void update(Vector3d trans, NDTFrame* const new_frame);
    void addPoint(Vector2d& point);
    void print();
    void build();
    inline int getCellIndex(Vector2d point);
    Vector3d align(Vector3d initial_guess, NDTFrame* const new_frame);
    void saveImage(const char* const filename, unsigned char density = 50);
    void addPose(Vector3d pose);
    void resetPoints();
};

extern double cost_function(Vector3d trans, NDTFrame* const ref_frame, NDTFrame* const new_frame);

#endif // NDTFRAME_H
