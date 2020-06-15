#include "ndtpso_slam/ndtframe.h"
#include "ndtpso_slam/core.h"
#include <cstdio>
#include <opencv/cv.hpp>
#include <opencv/cvwimage.h>
#include <opencv/ml.h>

NDTFrame::NDTFrame(Vector3d trans, unsigned short width, unsigned short height, double cell_side, bool calculate_cells_params
#if BUILD_OCCUPANCY_GRID
    ,
    double occupancy_grid_cell_size
#endif
    )
    : _trans(trans)
    , width(width)
    , height(height)
    , cell_side(cell_side)
{
    this->built = false;
    this->widthNumOfCells = uint16_t(ceil(width / cell_side));
    this->heightNumOfCells = uint16_t(ceil(height / cell_side));
    this->numOfCells = widthNumOfCells * heightNumOfCells;
    this->cells = vector<NDTCell>(this->numOfCells, NDTCell(calculate_cells_params));

#if BUILD_OCCUPANCY_GRID
    // Initializing the occupancy grid,
    this->_occupancy_grid.cell_size = occupancy_grid_cell_size;

    // I case of zero size cell, all operations on the occupancy grid are ommited,
    // This saves some computing time when OG aren't needed (like in intermediate
    // frames used for loading laser data and matching)
    if (occupancy_grid_cell_size > 0.) {
        this->_occupancy_grid.width = uint32_t(ceil(width / occupancy_grid_cell_size));
        this->_occupancy_grid.height = uint32_t(ceil(height / occupancy_grid_cell_size));
        this->_occupancy_grid.count = this->_occupancy_grid.width * this->_occupancy_grid.height;
        this->_occupancy_grid.og = vector<int8_t>(this->_occupancy_grid.count, 0);
    }
#endif

    /*
    // TODO: Needs to be generic !!
    if (positive_only) {
        this->_x_min = -7.2; // <-- here
        this->_x_max = width - 8.2; // <-- and here!
    } else {
        this->_x_min = -width / 2.;
        this->_x_max = width / 2.;
    }

    Deleted `positive_only` parameter, now this is simply:
    */

    this->_x_min = -width / 2.;
    this->_x_max = width / 2.;
    this->_y_min = -height / 2.;
    this->_y_max = height / 2.;
}

void NDTFrame::build()
{
    uint32_t n = static_cast<uint32_t>(floor(this->cell_side / this->_occupancy_grid.cell_size));

    for (unsigned int i = 0; i < this->numOfCells; ++i) {
        auto current_cell = &this->cells[i];

        if (current_cell->created) {
            current_cell->build();

#if BUILD_OCCUPANCY_GRID
            if (this->_occupancy_grid.cell_size > 0.) {
                uint32_t ind_x, ind_y;

                ind_x = i % this->widthNumOfCells;
                ind_y = i / this->heightNumOfCells;

                for (uint32_t j = 0; j < n; ++j) {
                    for (uint32_t k = 0; k < n; ++k) {
                        double x, y;

                        x = ((ind_x * n + j) * this->_occupancy_grid.cell_size + this->_occupancy_grid.cell_size / 2.) - (this->width / 2);
                        y = ((ind_y * n + k) * this->_occupancy_grid.cell_size + this->_occupancy_grid.cell_size / 2.) - (this->height / 2);

                        auto p = current_cell->normalDistribution(Vector2d(x, y)) /* - 0.5*/;

                        if (p > 0.) {
                            this->_occupancy_grid.og[(ind_x * n + j) + this->_occupancy_grid.height * (ind_y * n + k)] = int8_t(p * 100.);
                        }
                    }
                }
            }
#endif
        }
    }

    this->built = true;
}

#if false
void NDTFrame::transform(Vector3d trans)
{
    if (!trans.isZero(1e-6)) {
        vector<NDTCell>* old_cells = &this->cells;

        this->cells = vector<NDTCell>(this->numOfCells);

        for (unsigned int i = 0; i < this->numOfCells; ++i)
            if ((*old_cells)[i].created)
                for (unsigned int j = 0; j < this->cells[i].points->size(); ++j) {
                    Vector2d new_point = transform_point(this->cells[i].points[j], trans);
                    this->addPoint(new_point);
                }

        delete old_cells;

        this->built = false;
    }
}
#endif

// Initialize the cell from laser data according to the device sensibility and
// the minimum angle
void NDTFrame::loadLaser(vector<float> const& laser_data, float const& min_angle, float const& angle_increment, float const& max_range)
{
    this->built = false;
    unsigned int n = static_cast<unsigned int>(laser_data.size());

#if USING_TRANS
    // Define a function 'f' to do transformation if needed
    Vector2d (*trans_func)(Vector2d&, Vector3d&) = nullptr;

    if (!this->_trans.isZero(1e-6))
        trans_func = &transform_point;
#endif

    float theta;
#if PREFER_FRONTAL_POINTS
    float delta_theta = 0.f;
#endif

    // For each element in the laser vector, get his index (i) and it's
    // corresponding (theta)
    // according to the sensibility and the minimum angle
    for (unsigned int i = 0; i < n; ++i) {
        if ((laser_data[i] < max_range) && (laser_data[i] > LASER_IGNORE_EPSILON)) {
            theta = index_to_angle(i, angle_increment, min_angle);
#if PREFER_FRONTAL_POINTS
            delta_theta += sinf(theta);

            if (fabsf(delta_theta) > .5f) {
#endif
                Vector2d point = laser_to_point(laser_data[i], theta);

#if USING_TRANS
                if (trans_func)
                    point = trans_func(point, this->_trans);
#endif
                this->addPoint(point);
#if PREFER_FRONTAL_POINTS
                delta_theta = 0.f;
            }
#endif
        }
    }
}

void NDTFrame::update(Vector3d trans, NDTFrame* const new_frame)
{
    this->built = false; // Set 'built' flag to false to rebuild the cell if needed

    for (unsigned int i = 0; i < new_frame->cells.size(); ++i)
        if (new_frame->cells[i].created)
            for (unsigned int j = 0; j < new_frame->cells[i].points[0].size(); ++j) {
                Vector2d point = transform_point(new_frame->cells[i].points[0][j], trans);
                this->addPoint(point);
            }
}

void NDTFrame::addPose(double timestamp, Vector3d pose, Vector3d odom)
{
    // Used only for saving the global map image (if any), for scan matching; there is no need for this. Useful for debug
    this->_timestamps.push_back(timestamp);
    this->_poses.push_back(pose);
    this->_odoms.push_back(odom);
}

void NDTFrame::resetCells()
{
    unsigned int n = unsigned(this->cells.size());
    for (unsigned int i = 0; i < n; ++i)
        this->cells[i].reset();
}

// Add the given point 'pt' to it's corresponding cell
void NDTFrame::addPoint(Vector2d& point)
{
    // Get the cell index in the list
    int cell_index = this->getCellIndex(point, this->widthNumOfCells, this->cell_side);

    // If the point is contained in the frame borders and it's not at the origin
    if (-1 != cell_index) {
        // And then, append the point to its cell points list
        this->cells[static_cast<size_t>(cell_index)].addPoint(point);

        this->built = false; // Set 'built' flag to false to rebuild the cell if needed
    }

#if BUILD_OCCUPANCY_GRID && false
    if (this->_occupancy_grid.cell_size > 0.) {
        cell_index = this->getCellIndex(point,
            static_cast<int>(this->_occupancy_grid.width),
            this->_occupancy_grid.cell_size);
        if (-1 != cell_index) {
            this->_occupancy_grid.og[static_cast<unsigned int>(cell_index)]++;
        }
    }
#endif
}

// volatile const char *(*signal(int const * b, void (*fp)(int*)))(int**); // Just for fun!

int NDTFrame::getCellIndex(Vector2d point, int grid_width, double cell_side)
{
    // If the point is contained inside the FRAME borders
    if ((point.x() > this->_x_min) && (point.x() < this->_x_max)
        && (point.y() > this->_y_min) && (point.y() < this->_y_max)) {
        // Then return the index of the its corresponding CELL
        return static_cast<int>(floor((point.x() + (this->width / 2.)) / cell_side)
            + grid_width * (floor((point.y() + (this->height / 2.)) / cell_side)));
    } else {
        return -1;
    }
}

Vector3d NDTFrame::align(Vector3d initial_guess, NDTFrame* const new_frame)
{
    Vector3d deviation = Vector3d(.1, .1, 3.1415E-3); // Used to UNIFORMLY distribute the initial particles
    return pso_optimization(initial_guess, this, new_frame, PSO_ITERATIONS, deviation);
}

void NDTFrame::dumpMap(const char* const filename, bool save_poses, bool save_points, bool save_image, short density
#if BUILD_OCCUPANCY_GRID
    ,
    bool save_occupancy_grid
#endif
)
{
    // Save the points, poses & odoms to an image, useful for debugging!
    int size_x = this->width * density, // density in "pixel per meter"
        size_y = this->height * density;

    FILE *hndl_poses = nullptr, *hndl_points = nullptr;
    char output_filename[250];

    if (save_poses) {
        sprintf(output_filename, "%s.pose.csv", filename);
        hndl_poses = fopen(output_filename, "w");
        if (hndl_poses)
            fprintf(hndl_poses, "timestamp,xP,yP,thP,xO,yO,thO\n");
    }

    if (save_points) {
        sprintf(output_filename, "%s.map.csv", filename);
        hndl_points = fopen(output_filename, "w");
        if (hndl_points)
            fprintf(hndl_points, "x,y\n");
    }

    if ((save_poses && !hndl_poses) || (save_points && !hndl_points)) {
        printf("%s: Cannot open files, cannot save!\n ", __func__);
        return;
    }

    int counter = 0;

    cv::Mat img(size_x, size_y, CV_8UC3, cv::Scalar::all(255));

    // cv::Mat img_dist(size_x, size_y, CV_8UC3, cv::Scalar::all(0)); // To plot the normal distribution

    // Draw a grid (using `density` as increment, we draw a line each 1 meter)
    for (int i = 0; i < size_x; i += density) {
        cv::line(img, cv::Point(i, 0), cv::Point(i, size_y), cv::Scalar(180, 180, 180));
        cv::line(img, cv::Point(0, i), cv::Point(size_x, i), cv::Scalar(180, 180, 180));
    }

    // Draw and dump 2D points
    for (unsigned int i = 0; i < this->numOfCells; ++i) {
        for (unsigned int j = 0; j < this->cells[i].points[0].size(); ++j) {

            int x = (size_x / 2) + static_cast<int>(this->cells[i].points[0][j].x() * density);
            int y = (size_y / 2) - static_cast<int>(this->cells[i].points[0][j].y() * density);

            cv::circle(img, cv::Point(x, y), 1, cv::Scalar(0));

            if (save_points) {
                fprintf(hndl_points, "%.5f,%.5f\n", this->cells[i].points[0][j].x(), this->cells[i].points[0][j].y());
            }
        }
    }

    // Draw and dump poses and odometries
    for (unsigned i = 0; i < this->_odoms.size(); ++i) {
        int x = (size_x / 2) + static_cast<int>(this->_odoms[i].x() * density);
        int y = (size_y / 2) - static_cast<int>(this->_odoms[i].y() * density);
        int dx = static_cast<int>(.5 * cos(-this->_odoms[i].z()) * density);
        int dy = static_cast<int>(.5 * sin(-this->_odoms[i].z()) * density);

        if (0 == counter) {
            cv::line(img, cv::Point(x, y), cv::Point(x + dx, y + dy), cv::Scalar(100, 50, 0));
        }

        cv::circle(img, cv::Point(x, y), 2, cv::Scalar(255, 0, 0));

        x = (size_x / 2) + static_cast<int>(this->_poses[i].x() * density);
        y = (size_y / 2) - static_cast<int>(this->_poses[i].y() * density);
        dx = static_cast<int>(.5 * cos(-this->_poses[i].z()) * density);
        dy = static_cast<int>(.5 * sin(-this->_poses[i].z()) * density);

        if (0 == counter) {
            cv::line(img, cv::Point(x, y), cv::Point(x + dx, y + dy), cv::Scalar(40, 40, 80));
        }

        cv::circle(img, cv::Point(x, y), 2, cv::Scalar(0, 0, 255));

        counter = (counter + 1) % 5;

        if (save_points) {
            fprintf(hndl_poses, "%.6f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
                this->_timestamps[i],
                this->_poses[i].x(), this->_poses[i].y(), this->_poses[i].z(),
                this->_odoms[i].x(), this->_odoms[i].y(), this->_odoms[i].z());
        }
    }

    if (save_poses)
        fclose(hndl_poses);

    if (save_points)
        fclose(hndl_points);

    if (save_poses || save_points) {
        // Save the .gnuplot file to plot the outputs
        sprintf(output_filename, "%s.gnuplot", filename);
        hndl_poses = fopen(output_filename, "w");

        fprintf(hndl_poses,
            "set datafile separator ','\n"
            "set key autotitle columnhead\n"
            "set size ratio -1\n"
            "plot '%s.map.csv' title 'Map (from front scans)' with points pointsize 0.2 "
            "pointtype 5 linecolor rgb '#555555', \\\n"
            "'%s.pose.csv' using 2:3 title 'Pose (back lidar)' with linespoints linewidth 0.7 "
            "pointtype 6 pointsize 0.7 linecolor rgb '#ff0000', \\\n"
            "'%s.pose.csv' using 5:6 title 'Odometry' with linespoints linewidth 0.7 pointtype "
            "6 pointsize 0.7 linecolor rgb '#0000ff'\n"
            "pause 1000\n",
            filename,
            filename,
            filename);

        fclose(hndl_poses);
    }

    if (save_image) {
        sprintf(output_filename, "%s-w%d-PSOitr%d-PSOpop%d-%dx%d-c%.2f-%dppm.png",
            filename, NDT_WINDOW_SIZE, PSO_ITERATIONS, PSO_POPULATION_SIZE,
            this->width, this->height, this->cell_side, density);

        imwrite(output_filename, img);
    }

    if (save_occupancy_grid) {
        cv::Mat img_og(static_cast<int>(this->_occupancy_grid.width),
            static_cast<int>(this->_occupancy_grid.height),
            CV_8U, cv::Scalar::all(255));

        for (unsigned int i = 0; i < this->_occupancy_grid.count; ++i) {
            if (this->_occupancy_grid.og[i] > 0) {
                img_og.at<uint8_t>(
                    int(this->_occupancy_grid.width - (i / this->_occupancy_grid.width)),
                    int(i % this->_occupancy_grid.height))
                    = uint8_t(255.0 - this->_occupancy_grid.og[i] * 2.55);
            }
        }

        sprintf(output_filename, "%s-%dxh%d-c%.2f-occupancy-grid.png",
            filename, this->_occupancy_grid.width, this->_occupancy_grid.height, this->_occupancy_grid.cell_size);

        imwrite(output_filename, img_og);
    }
}
