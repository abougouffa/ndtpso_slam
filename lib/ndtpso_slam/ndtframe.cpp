#include "ndtpso_slam/ndtframe.h"
#include "ndtpso_slam/core.h"
#include <cstdio>
#include <utility>

#ifdef OPENCV_FOUND
#include <opencv/cv.hpp>
#include <opencv/cvwimage.h>
#include <opencv/ml.h>
#endif

#ifndef MIN
#define MIN(a, b) ((a) > (b) ? (b) : (a))
#endif

#ifndef MAX
#define MAX(a, b) ((a) < (b) ? (b) : (a))
#endif

NDTFrame::NDTFrame(Vector3d trans,
    unsigned short width,
    unsigned short height,
    double cell_side,
    bool calculate_cells_params,
    NDTPSOConfig config
#if BUILD_OCCUPANCY_GRID
    ,
    double occupancy_grid_cell_size
#endif
    )
    : s_trans(std::move(trans))
    , s_config(std::move(config))
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
    this->s_occupancy_grid.cell_size = occupancy_grid_cell_size;

    // I case of zero size cell, all operations on the occupancy grid are ommited,
    // This saves some computing time when OG aren't needed (like in intermediate
    // frames used for loading laser data and matching)
    if (occupancy_grid_cell_size > 0.) {
        this->s_occupancy_grid.width = uint32_t(ceil(width / occupancy_grid_cell_size));
        this->s_occupancy_grid.height = uint32_t(ceil(height / occupancy_grid_cell_size));
        this->s_occupancy_grid.count = this->s_occupancy_grid.width * this->s_occupancy_grid.height;
        this->s_occupancy_grid.og = vector<int8_t>(this->s_occupancy_grid.count, 0);
    }
#endif

#if false
    /*
    TODO: Needs to be generic !!
    */

    if (positive_only) {
        this->s_x_min = -7.2; // <-- here
        this->s_x_max = width - 8.2; // <-- and here!
    } else {
#endif
    this->s_x_min = -width / 2.;
    this->s_x_max = width / 2.;

#if false
    }
#endif

    this->s_y_min = -height / 2.;
    this->s_y_max = height / 2.;
}

void NDTFrame::build()
{
    auto og_cells_per_cell = static_cast<uint32_t>(floor(this->cell_side / this->s_occupancy_grid.cell_size));

    for (unsigned int i = 0; i < this->numOfCells; ++i) {
        auto current_cell = &this->cells[i];

        if (current_cell->created) {
            current_cell->build();

#if BUILD_OCCUPANCY_GRID
            if (this->s_occupancy_grid.cell_size > 0.) {
                uint32_t cell_x_ind = i % this->widthNumOfCells,
                         cell_y_ind = i / this->heightNumOfCells;

                for (uint32_t j = 0; j < og_cells_per_cell; ++j) {
                    for (uint32_t k = 0; k < og_cells_per_cell; ++k) {
                        double x_c = ((cell_x_ind * og_cells_per_cell + j)
                                             * this->s_occupancy_grid.cell_size
                                         + this->s_occupancy_grid.cell_size / 2.)
                            - (this->width / 2.);

                        double y_c = ((cell_y_ind * og_cells_per_cell + k)
                                             * this->s_occupancy_grid.cell_size
                                         + this->s_occupancy_grid.cell_size / 2.)
                            - (this->height / 2.);

                        auto p = current_cell->normalDistribution(Vector2d(x_c, y_c)) /* - 0.5*/;

                        if (p > 0.) {
                            uint32_t og_x_ind = cell_x_ind * og_cells_per_cell + j;
                            uint32_t og_y_ind = cell_y_ind * og_cells_per_cell + k;

                            /*
                             * TODO: move this outside to store the max/min x and y, as coordinates not as indexes,
                             * so the same info can be used with the point cloud map
                             */
                            this->s_occupancy_grid.min_x_ind = MIN(og_x_ind, this->s_occupancy_grid.min_x_ind);
                            this->s_occupancy_grid.max_x_ind = MAX(og_x_ind, this->s_occupancy_grid.max_x_ind);
                            this->s_occupancy_grid.min_y_ind = MIN(og_y_ind, this->s_occupancy_grid.min_y_ind);
                            this->s_occupancy_grid.max_y_ind = MAX(og_y_ind, this->s_occupancy_grid.max_y_ind);

                            this->s_occupancy_grid.og[og_x_ind + this->s_occupancy_grid.height * og_y_ind] = int8_t(p * 100.);
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
void NDTFrame::loadLaser(vector<float> const& laser_data,
    float const& min_angle,
    float const& angle_increment,
    float const& max_range)
{
    this->built = false;
    auto n = static_cast<unsigned int>(laser_data.size());

#if USING_TRANS
    // Define a function 'f' to do transformation if needed
    Vector2d (*trans_func)(const Vector2d&, const Vector3d&) = nullptr;

    if (!this->s_trans.isZero(1e-6))
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
        if ((laser_data[i] < max_range) && (laser_data[i] > this->s_config.laserIgnoreEpsilon)) {
            theta = index_to_angle(i, angle_increment, min_angle);
#if PREFER_FRONTAL_POINTS
            delta_theta += sinf(theta);

            if (fabsf(delta_theta) > .5f) {
#endif
                Vector2d point = laser_to_point(laser_data[i], theta);

#if USING_TRANS
                if (trans_func)
                    point = trans_func(point, this->s_trans);
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

    for (auto& new_frame_cell : new_frame->cells) {
        if (new_frame_cell.created) {
            for (auto& point : new_frame_cell.points[0]) {
                Vector2d pt = transform_point(point, trans);
                this->addPoint(pt);
            }
        }
    }
}

void NDTFrame::addPose(double timestamp, const Vector3d& pose, const Vector3d& odom)
{
    // Used only for saving the global map image (if any), for scan matching; there is no need for this. Useful for debug
    this->s_timestamps.push_back(timestamp);
    this->s_poses.push_back(pose);
    this->s_odoms.push_back(odom);
}

void NDTFrame::resetCells()
{
    auto n = static_cast<unsigned int>(this->cells.size());
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
    if (this->s_occupancy_grid.cell_size > 0.) {
        cell_index = this->getCellIndex(point,
            static_cast<int>(this->s_occupancy_grid.width),
            this->s_occupancy_grid.cell_size);
        if (-1 != cell_index) {
            this->s_occupancy_grid.og[static_cast<unsigned int>(cell_index)]++;
        }
    }
#endif
}

// volatile const char *(*signal(int const * b, void (*fp)(int*)))(int**); // Just for fun!

int NDTFrame::getCellIndex(Vector2d point, int grid_width, double cell_side)
{
    // If the point is contained inside the FRAME borders
    if ((point.x() > this->s_x_min) && (point.x() < this->s_x_max)
        && (point.y() > this->s_y_min) && (point.y() < this->s_y_max)) {
        // Then return the index of the its corresponding CELL
        return static_cast<int>(floor((point.x() + (this->width / 2.)) / cell_side)
            + grid_width * (floor((point.y() + (this->height / 2.)) / cell_side)));
    }

    return -1;
}

Vector3d NDTFrame::align(Vector3d initial_guess, const NDTFrame* const new_frame)
{
    Vector3d deviation = Vector3d(.1, .1, 3.1415E-3); // Used to UNIFORMLY distribute the initial particles
    return pso_optimization(std::move(initial_guess), this, new_frame, deviation);
}

void NDTFrame::dumpMap(const char* filename, bool save_poses, bool save_points, bool save_image, short density
#if BUILD_OCCUPANCY_GRID
    ,
    bool save_occupancy_grid
#endif
)
{
    // Save the points, poses & odoms to an image, useful for debugging!
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

#ifdef OPENCV_FOUND
    int size_x = this->width * density, // density in "pixel per meter"
        size_y = this->height * density;

    int counter = 0;

    cv::Mat img(size_x, size_y, CV_8UC3, cv::Scalar::all(255));

    // cv::Mat img_dist(size_x, size_y, CV_8UC3, cv::Scalar::all(0)); // To plot the normal distribution

    // Draw a grid (using `density` as increment, we draw a line each 1 meter)
    for (int i = 0; i < size_x; i += density) {
        cv::line(img, cv::Point(i, 0), cv::Point(i, size_y), cv::Scalar(180, 180, 180));
        cv::line(img, cv::Point(0, i), cv::Point(size_x, i), cv::Scalar(180, 180, 180));
    }
#endif

    // Draw and dump 2D points
    //for (unsigned int i = 0; i < this->numOfCells; ++i) {
    for (auto& cell : this->cells) {
        for (auto& points : cell.points) {
            for (auto& point : points) {
                // for (unsigned int j = 0; j < points.size(); ++j) {
#ifdef OPENCV_FOUND
                int x = (size_x / 2) + static_cast<int>(point.x() * density);
                int y = (size_y / 2) - static_cast<int>(point.y() * density);
                cv::circle(img, cv::Point(x, y), 1, cv::Scalar(0));
#endif
                if (save_points) {
                    fprintf(hndl_points, "%.5f,%.5f\n", point.x(), point.y());
                    // }
                }
            }
        }
    }

    // Draw and dump poses and odometries
    for (unsigned int i = 0; i < this->s_odoms.size(); ++i) {
#ifdef OPENCV_FOUND
        auto x = (size_x / 2) + static_cast<int>(this->s_odoms[i].x() * density),
             y = (size_y / 2) - static_cast<int>(this->s_odoms[i].y() * density),
             dx = static_cast<int>(.5 * cos(-this->s_odoms[i].z()) * density),
             dy = static_cast<int>(.5 * sin(-this->s_odoms[i].z()) * density);

        if (0 == counter) {
            cv::line(img, cv::Point(x, y), cv::Point(x + dx, y + dy), cv::Scalar(100, 50, 0));
        }

        cv::circle(img, cv::Point(x, y), 2, cv::Scalar(255, 0, 0));

        x = (size_x / 2) + static_cast<int>(this->s_poses[i].x() * density);
        y = (size_y / 2) - static_cast<int>(this->s_poses[i].y() * density);
        dx = static_cast<int>(.5 * cos(-this->s_poses[i].z()) * density);
        dy = static_cast<int>(.5 * sin(-this->s_poses[i].z()) * density);

        if (0 == counter) {
            cv::line(img, cv::Point(x, y), cv::Point(x + dx, y + dy), cv::Scalar(40, 40, 80));
        }

        cv::circle(img, cv::Point(x, y), 2, cv::Scalar(0, 0, 255));

        counter = (counter + 1) % 5;
#endif
        if (save_points) {
            fprintf(hndl_poses, "%.6f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
                this->s_timestamps[i],
                this->s_poses[i].x(), this->s_poses[i].y(), this->s_poses[i].z(),
                this->s_odoms[i].x(), this->s_odoms[i].y(), this->s_odoms[i].z());
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

#ifdef OPENCV_FOUND
    if (save_image) {
        sprintf(output_filename, "%s-w%d-PSOitr%d-PSOpop%d-%dx%d-c%.2f-%dppm.png",
            filename, NDT_WINDOW_SIZE, this->s_config.psoConfig.iterations, this->s_config.psoConfig.populationSize,
            this->width, this->height, this->cell_side, density);

        imwrite(output_filename, img);
    }

    if (save_occupancy_grid) {
        uint32_t real_width = this->s_occupancy_grid.max_x_ind - this->s_occupancy_grid.min_x_ind,
                 real_heigth = this->s_occupancy_grid.max_y_ind - this->s_occupancy_grid.min_y_ind;

        cv::Mat img_og(static_cast<int>(real_heigth), static_cast<int>(real_width), CV_8U, cv::Scalar::all(255));

        for (uint32_t i = this->s_occupancy_grid.min_x_ind; i <= this->s_occupancy_grid.max_x_ind; ++i) {
            for (uint32_t j = this->s_occupancy_grid.min_y_ind; j <= this->s_occupancy_grid.max_y_ind; ++j) {
                size_t ind = i + this->s_occupancy_grid.height * j;
                if (this->s_occupancy_grid.og[ind] > 0) {
                    img_og.at<uint8_t>(
                        int(real_heigth - (j - this->s_occupancy_grid.min_y_ind)),
                        int(i - this->s_occupancy_grid.min_x_ind))
                        = uint8_t(255.0 - this->s_occupancy_grid.og[i + this->s_occupancy_grid.height * j] * 2.55);
                }
            }
        }

        sprintf(output_filename, "%s-%dx%d-cell%.2fm-occupancy-grid.png",
            filename, this->s_occupancy_grid.width, this->s_occupancy_grid.height, this->s_occupancy_grid.cell_size);

        imwrite(output_filename, img_og);
    }
#endif
}
