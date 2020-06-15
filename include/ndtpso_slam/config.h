#ifndef CONFIG_H
#define CONFIG_H

// Default values
#define NDT_WINDOW_SIZE 100
#define NDT_MAX_POINTS_PER_CELL 100 // Unused now, to be used to limit the points/cell by a circular buffer, to ensure const memory
#define LASER_IGNORE_EPSILON 0.1f // Ignore points around the origin with 10cm
#define USING_TRANS false
#define PREFER_FRONTAL_POINTS false // Disabled
#define BUILD_OCCUPANCY_GRID true

#define USE_LOGGER false

#if USE_LOGGER
#define LOGGER_BUFFER_SIZE_LINES 50
#endif

// PSO parameters
#define PSO_ITERATIONS 60
#define PSO_POPULATION_SIZE 25
#define PSO_W_DUMPING_COEF 1.
#define PSO_W .8
#define PSO_C1 2.
#define PSO_C2 2.

struct NdtPsoConfig {
    int psoNumIterations;
    int psoPopulationSize;
    int ndtWindowSize;
    float laserIgnoreEpsilon;

    NdtPsoConfig()
    {
        // psoNumIterations = PSO_ITERATIONS;
        // psoPopulationSize = PSO_POPULATION_SIZE;
        // ndtWindowSize = NDT_WINDOW_SIZE;
        // laserIgnoreEpsilon = LASER_IGNORE_EPSILON;
    }
};

#endif // CONFIG_H
