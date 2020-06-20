#ifndef CONFIG_H
#define CONFIG_H

// Default values
#define NDT_MAX_POINTS_PER_CELL 20
#define LASER_IGNORE_EPSILON 0.1f // Ignore points around the origin with 10cm

#define NDT_WINDOW_SIZE 100
#define USING_TRANS true
#define PREFER_FRONTAL_POINTS false // Disabled
#define BUILD_OCCUPANCY_GRID true

#define USE_LOGGER false

#if USE_LOGGER
#define LOGGER_BUFFER_SIZE_LINES 50
#endif

// PSO parameters
#define PSO_ITERATIONS 60
#define PSO_POPULATION_SIZE 35
#define PSO_W_DUMPING_COEF 1.
#define PSO_W .8
#define PSO_C1 2.
#define PSO_C2 2.

struct PSOConfig {
    unsigned int iterations{ PSO_ITERATIONS };
    unsigned int populationSize{ PSO_POPULATION_SIZE };
    int num_threads{ -1 };
    struct {
        double w{ PSO_W };
        double c1{ PSO_C1 };
        double c2{ PSO_C2 };
        double w_dumping{ PSO_W_DUMPING_COEF };
    } coeff;
};

struct NDTPSOConfig {
    PSOConfig psoConfig;
    // unsigned int ndtWindowSize{ NDT_WINDOW_SIZE };
    // unsigned int maxPointsPerCell{ NDT_MAX_POINTS_PER_CELL };
    float laserIgnoreEpsilon{ LASER_IGNORE_EPSILON };
};

#endif // CONFIG_H
