#ifndef CONFIG_H
#define CONFIG_H

// Default values
#define NDT_WINDOW_SIZE 100
#define LASER_IGNORE_EPSILON 0.1f // Ignore points around the origin with 10cm
#define USING_TRANS true
#define PREFER_FRONTAL_POINTS false // Disabled

// PSO parameters
#define PSO_ITERATIONS 70
#define PSO_POPULATION_SIZE 40
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
