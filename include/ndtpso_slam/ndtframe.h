#ifndef NDTFRAME_H
#define NDTFRAME_H

#include "ndtpso_slam/ndtcell.h"
#include <eigen3/Eigen/Core>
#include <vector>

using namespace Eigen;
using std::vector;

class NDTFrame {
private:
    Vector3d s_trans;
    vector<Vector3d> s_poses, s_odoms;
    vector<double> s_timestamps;
    double s_x_min, s_x_max, s_y_min, s_y_max;
    NdtPsoConfig s_config;

#if BUILD_OCCUPANCY_GRID
    struct {
        uint32_t count;
        uint32_t width;
        uint32_t height;
        double cell_size;
        vector<int8_t> og;
    } s_occupancy_grid;
#endif

public:
    uint16_t width, height, widthNumOfCells, heightNumOfCells;
    vector<NDTCell> cells;
    bool built;
    unsigned int numOfCells;
    double cell_side;
    NDTFrame(Vector3d trans, unsigned short width = 20, unsigned short height = 20, double cell_side = 1.0, bool calculate_cells_params = true
#if BUILD_OCCUPANCY_GRID
        ,
        double occupancy_grid_cell_size = .0
#endif
    );
    // void transform(Vector3d trans);
    void loadLaser(const vector<float>& laser_data, const float& min_angle, const float& angle_increment, const float& max_range);
    void update(Vector3d trans, NDTFrame* const new_frame);
    void addPoint(Vector2d& point);
    inline void setTrans(Vector3d trans) { this->s_trans = trans; }
#if defined(DEBUG) && DEBUG
    void print();
#endif
#if false
    void transform(Vector3d trans);
#endif
    void build();
    int getCellIndex(Vector2d point, int grid_width, double cell_side);
    Vector3d align(Vector3d initial_guess, NDTFrame* const new_frame);
    void dumpMap(const char* const filename, bool save_poses = true, bool save_points = true, bool save_image = true, short density = 50
#if BUILD_OCCUPANCY_GRID
        ,
        bool save_occupancy_grid = true
#endif
    );
    void addPose(double timestamp, Vector3d pose, Vector3d odom = Vector3d::Zero());
    void resetCells();
};

#endif // NDTFRAME_H
