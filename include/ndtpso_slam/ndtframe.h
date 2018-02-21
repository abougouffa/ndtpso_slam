#ifndef NDTFRAME_H
#define NDTFRAME_H

#include "ndtpso_slam/ndtcell.h"
#include <eigen3/Eigen/Core>
#include <vector>

using namespace Eigen;
using std::vector;

class NdtFrame {
private:
    Vector3d _trans;

public:
    uint16_t width, height, widthNumOfCells, heightNumOfCells;
    vector<NdtCell> cells;
    bool built;
    unsigned int numOfCells;
    double cell_side;
    NdtFrame(Vector3d trans, unsigned short width = 20, unsigned short height = 20, double cell_side = 1.0);
    void transform(Vector3d trans);
    void loadLaser(vector<float>& laser_data, float min_angle = static_cast<float>(-M_PI_2), float max_angle = static_cast<float>(M_PI_2));
    void update(Vector3d trans, NdtFrame* const new_frame);
    void addPoint(Vector2d& point);
    void print();
    void build();
    inline int getCellIndex(Vector2d point);
    Vector3d align(Vector3d initial_guess, NdtFrame* const new_frame);
    void saveImage(const char* const filename, unsigned char density = 50);
};

extern double cost_function(Vector3d trans, NdtFrame* const ref_frame, NdtFrame* const new_frame);

#endif // NDTFRAME_H
