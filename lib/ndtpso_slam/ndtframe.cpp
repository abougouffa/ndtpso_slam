#include "ndtpso_slam/ndtframe.h"
#include "ndtpso_slam/core.h"
#include <CImg.h>
//#include "ndtpso_slam/particle.h"

using namespace cimg_library;

NdtFrame::NdtFrame(Vector3d trans, uint16_t width, uint16_t height, double cell_side)
    : _trans(trans)
    , width(width)
    , height(height)
    , cell_side(cell_side)
{
    this->built = false;
    this->widthNumOfCells = uint16_t(ceil(width / cell_side));
    this->heightNumOfCells = uint16_t(ceil(height / cell_side));
    this->numOfCells = widthNumOfCells * heightNumOfCells;
    this->cells = vector<NdtCell>(this->numOfCells);
}

void NdtFrame::print()
{
    for (unsigned short i = 0; i < this->numOfCells; ++i) {
        this->cells[i].print(i);
    }
}

void NdtFrame::build()
{
    if (this->built) {
        return;
    }

    for (unsigned short i = 0; i < this->numOfCells; ++i) {
        if (this->cells[i].created)
            this->cells[i].build();
    }

    this->built = true;
}

// Initialize the cell from laser data according to the device sensibility and
// the minimum angle
void NdtFrame::transform(Vector3d trans)
{
    if (!trans.isZero(1e-6)) {
        vector<NdtCell>* old_cells = &this->cells;

        this->cells = vector<NdtCell>(this->numOfCells);

        for (unsigned int i = 0; i < this->numOfCells; ++i) {
            if ((*old_cells)[i].created) {
                for (unsigned short j = 0; j < this->cells[i].points.size(); ++j) {
                    Vector2d new_point = transform_point(this->cells[i].points[j], trans);
                    this->addPoint(new_point);
                }
            }
        }

        delete old_cells;

        this->built = false;
    }
}

void NdtFrame::loadLaser(vector<float> laser_data, float min_angle, float max_angle)
{
    unsigned short n = static_cast<unsigned short>(laser_data.size());

    float sensibility = (max_angle - min_angle) / (n - 1.); // Caclulate the sensor sensibility

    // Define a function 'f' to do transformation if needed
    Vector2d (*f)(Vector2d&, Vector3d&) = NULL;

    if (!this->_trans.isZero(1e-6))
        f = &transform_point;

    float theta;

    // For each element in the laser vector, get his index (i) and it's
    // corresponding (theta)
    // according to the sensibility and the minimum angle
    for (unsigned short i = 0; i < n; ++i) {
        theta = INDEX_TO_ANGLE(i, sensibility, min_angle);
        Vector2d point = LASER_TO_POINT(laser_data[i], theta);

        if (f)
            point = f(point, this->_trans);

        this->addPoint(point);
    }
}

void NdtFrame::update(Vector3d trans, NdtFrame* const new_frame)
{
    for (unsigned int i = 0; i < new_frame->cells.size(); ++i) {
        if (new_frame->cells[i].created) {
            for (unsigned int j = 0; j < new_frame->cells[i].points.size(); ++j) {
                Vector2d point = transform_point(new_frame->cells[i].points[j], trans);
                this->addPoint(point);
            }
        }
    }
}

void NdtFrame::addPose(Vector3d pose)
{
    this->_poses.push_back(pose);
}

// TODO: Review me
// Add the given point 'pt' to it's corresponding cell
void NdtFrame::addPoint(Vector2d& point)
{
    // Get the cell index in the list
    int cell_index = this->getCellIndex(point);

    // If the point is contained in the frame borders and it's not at the origin
    if (cell_index != -1) {
        // And then, append the point to its cell points list
        this->cells[static_cast<unsigned int>(cell_index)].addPoint(point);

        this->built = false; // Set 'built' flag to false to rebuild the cell if needed
    }
}

// volatile const char *(*signal(int const * b, void (*fp)(int*)))(int**);

/* FIXME: Find a better implementation (negative values) */
int NdtFrame::getCellIndex(Vector2d point)
{
    // If the point in contained in the frame borders and it's not at the origin
    if ((point[0] >= 0)
        && (point[0] < this->width)
        && (point[1] > (-this->height / 2.))
        && (point[1] < (this->height / 2.))
        && ((point[0] > LASER_IGNORE_EPSILON) || (point[1] > LASER_IGNORE_EPSILON))) {
        return static_cast<int>(floor(point[0] / this->cell_side))
            + static_cast<int>(this->widthNumOfCells * (floor(((point[1] + (this->height / 2.)) / this->cell_side))));
    } else {
        return -1;
    }
}

// TODO: Test the cost function, (why it doesn't give the same value as the
// python implementation?!)
double cost_function(Vector3d trans, NdtFrame* const ref_frame, NdtFrame* const new_frame)
{
    if (!ref_frame->built)
        ref_frame->build();

    double trans_cost = 0.;

    for (unsigned int i = 0; i < new_frame->numOfCells; ++i) {
        for (unsigned int j = 0; j < new_frame->cells[i].points.size(); ++j) {
            Vector2d point = transform_point(new_frame->cells[i].points[j], trans);
            int index_in_ref_frame = ref_frame->getCellIndex(point);

            if ((index_in_ref_frame != -1) && ref_frame->cells[static_cast<unsigned int>(index_in_ref_frame)].isBuilt) {
                double point_probability = ref_frame->cells[static_cast<unsigned int>(index_in_ref_frame)]
                                               .normalDistribution(point);
                assert(1. >= point_probability || point_probability >= 0.);
                trans_cost -= static_cast<double>(point_probability);
            }
        }
    }

    return trans_cost;
}

Vector3d NdtFrame::align(Vector3d initial_guess, NdtFrame* const new_frame)
{
    assert(this->cell_side == new_frame->cell_side);
    Vector3d deviation;
    deviation << 1.5, .8, 3.E-1;
    return pso_optimization(initial_guess, this, new_frame, PSO_ITERATIONS, deviation);
}

void NdtFrame::saveImage(const char* const filename, unsigned char density)
{
    unsigned int size_x = this->width * density, // density in "pixel per meter"
        size_y = this->height * density,
                 size_z = 1,
                 numberOfColorChannels = 3;
    unsigned char initialValue = 0;

    CImg<unsigned char> image(size_x, size_y, size_z, numberOfColorChannels, initialValue);

    for (unsigned int i = 0; i < this->numOfCells; ++i) {
        for (unsigned int j = 0; j < this->cells[i].points.size(); ++j) {
            unsigned char point_color[] = { 255, 0, 0 }; // RGB

            int x = static_cast<int>(this->cells[i].points[j][0] * density);
            int y = static_cast<int>(this->cells[i].points[j][1] * density + (size_y / 2));

            image.draw_circle(x, y, 2, point_color);
            //            image.draw_point(x, y, randomColor);
        }
    }

    char save_filename[200];
    sprintf(save_filename, "%s-w%d-h%d-c%.2f-%dppm.png", filename, this->width, this->height, this->cell_side, density);
    image.save_png(save_filename, 3);
}
