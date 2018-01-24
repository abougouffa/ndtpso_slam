#include "ndtpso_slam/ndtframe.h"

bool operator==(const TransformParams &left, const TransformParams &right)
{
    return (left.x == right.x) && (left.y == right.y) && (left.theta == right.theta);
}


NdtFrame::NdtFrame(TransformParams trans, uint16_t width, uint16_t height, double cell_side):
    _trans(trans), _cell_side(cell_side), width(width), height(height)
{
    this->built = false;
    this->widthNumOfCells = uint16_t(floor(width / cell_side));
    this->heightNumOfCells = uint16_t(floor(height / cell_side));
    this->numOfCells = widthNumOfCells * heightNumOfCells;
    this->cells = vector<NdtCell>(this->numOfCells);
}


void NdtFrame::print() {
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


// Initialize the cell from laser data according to the device sensibility and the minimum angle
void NdtFrame::transform(TransformParams trans)
{
    if (trans.x != 0. && trans.y != 0. && trans.theta != 0.) {
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

        this->built = false;
    }
}


void NdtFrame::loadLaser(vector<double> laser_data, double min_angle, double max_angle)
{
    unsigned short n = static_cast<unsigned short>(laser_data.size());

    double sensibility = (max_angle - min_angle) / (n - 1.); // Caclulate the sensor sensibility

    // Define a function 'f' to do transformation if needed
    Vector2d (*f)(Vector2d, TransformParams) = NULL;

    if (this->_trans.x != 0. && this->_trans.y != 0. && this->_trans.theta != 0.)
        f = &transform_point;

    double theta;

    // For each element in the laser vector, get his index (i) and it's corresponding (theta)
    // according to the sensibility and the minimum angle
    for (unsigned short i = 0; i < n; ++i) {
        theta = INDEX_TO_ANGLE(i, sensibility, min_angle);
        Vector2d point = LASER_TO_POINT(laser_data[i], theta);

        if (f)
            point = f(point, this->_trans);

        this->addPoint(point);
    }
}


// Add the given point 'pt' to it's corresponding cell
void NdtFrame::addPoint(Vector2d &point)
{
    // Get the cell index in the list
    int cell_index = this->getCellIndex(point);

    // If the point in contained in the frame borders and it's not at the origin
    if (cell_index != -1) {
        // If the cell is not created yet (cells table created with 'calloc'), just create it
//        if (!(this->cells[static_cast<unsigned int>(cell_index)].created)) {
//            this->cells[static_cast<unsigned int>(cell_index)] = NdtCell();
//        }

        // And then, append the point to its cell points list
        this->cells[static_cast<unsigned int>(cell_index)].addPoint(point);

        this->built = false; // Set 'built' flag to false to rebuild the cell if needed
    }
}

//volatile const char *(*signal(int const * b, void (*fp)(int*)))(int**);

//def score(trans, ndt_frame_ref, ndt_frame_new):
double cost(TransformParams trans, NdtFrame * const ref_frame, NdtFrame * const new_frame)
{
    if (!ref_frame->built)
        ref_frame->build();

    double trans_cost = 0.;

    for (unsigned int i = 0; i < new_frame->numOfCells; ++i) {
        for (unsigned int j = 0; j < new_frame->cells[i].points.size(); ++j) {
            Vector2d point = transform_point(new_frame->cells[i].points[j], trans);
            int index_in_ref_frame = ref_frame->getCellIndex(point);

            if ((index_in_ref_frame != -1) && ref_frame->cells[static_cast<unsigned int>(index_in_ref_frame)].isBuilt) {
                double nd = ref_frame->cells[static_cast<unsigned int>(index_in_ref_frame)].normalDistribution(point);
//              assert (1. >= nd || nd >= 0.);
                trans_cost -= static_cast<double>(nd);
            }
        }
    }

    return trans_cost;
}
//    # The score function, return the negative of score to be used in optimization
//    if DEBUG: print('FUNC:: Trans -> ', trans)
//    if not ndt_frame_ref.built:
//        raise Exception("ndt_frame_ref must be built, call it's build() method before optimize")

//    scr = 0 # The score!

//    for key in ndt_frame_new.cell: # For each cell origin (key) in the new frame, do --->
//        for pt in ndt_frame_new.cell[key].points: # For each point in the new frame --->
//            point  = transform(pt, trans)
//            origin = origin_at(point, ndt_frame_ref.cell_side)

//            # Check if this cell origin exist in the reference frame and --->
//            # And if the cell in the reference frame is a NDT cell (have at least 3 points) --->
//            if ndt_frame_ref.cell.get(origin) and ndt_frame_ref.cell[origin].mean is not None:
//                q = point - ndt_frame_ref.cell[origin].mean # Calculate the q = point (in the new frame) - mean (of the same cell in the ref frame)
//                nd = ndt_frame_ref.cell[origin].normal_distribution(point) # Calculate the probability of pt in the ref cell
//                # if 1 < nd or nd < 0:
//                #     print("Abnomally ND={}".format(nd)) # This will fail if the probability if greater than 1 or lesser than 0
//                assert 1 >= nd >= 0
//                scr -= nd # Add the negative nd to score
//            # else:
//            #     scr += 1 # Like returning -1 as normal distribution

//    return scr # Return the score

/* TEST: Find a better implementation (negative values) */
int NdtFrame::getCellIndex(Vector2d point) {
    // If the point in contained in the frame borders and it's not at the origin
    if (point(0) < this->width &&
            (-this->height / 2.) < point(1) &&
            point(1) < (this->height / 2.) &&
            (point(0) > LASER_IGNORE_EPSILON || point(1) > LASER_IGNORE_EPSILON)) {
        return static_cast<int>(floor(point(0) / this->_cell_side))
                + static_cast<int>( this->widthNumOfCells * (floor((point(1) + (this->height / 2.) / this->_cell_side))));
    } else {
        return -1;
    }
}
