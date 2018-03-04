#include "ndtpso_slam/core.h"
#include "ndtpso_slam/ndtframe.h"
#include <cstdio>
#include <iostream>
#include <omp.h>

using std::cout;

struct Particle {
    Vector3d position, velocity, best_position;
    double best_cost;
    double cost;
    double pbest_average;

    Particle(Array3d mean, Array3d deviation, NDTFrame* const ref_frame, NDTFrame* const new_frame)
    {
        position = mean + (Array3d::Random() * deviation); // Randomly place the particles according to mean and deviation
        velocity << 0, 0, 0;

        cost = cost_function(position, ref_frame, new_frame);

        best_position = position;
        best_cost = cost;
        pbest_average = cost;
    }
};

Vector3d pso_optimization(Vector3d initial_guess, NDTFrame* const ref_frame, NDTFrame* const new_frame, unsigned int iters_num, Array3d deviation)
{
    double w = .8, w_damping_coef = 1., c1 = 2., c2 = 2.;
    Array3d zero_devi;
    zero_devi << 1E-4, 1E-4, 1E-5;

    //    vector<Particle> particles(num_of_particles - 1, Particle(initial_guess.array(), deviation, ref_frame, new_frame));
    vector<Particle> particles;

    // Add the initial estimation as global best
    Particle global_best(initial_guess.array(), zero_devi, ref_frame, new_frame);

    for (unsigned int i = 0; i < PSO_POPULATION_SIZE; ++i) {
        particles.push_back(Particle(initial_guess.array(), deviation, ref_frame, new_frame));

        if (particles[i].cost < global_best.best_cost) {
            //            global_best_index = i;
            global_best.best_cost = particles[i].best_cost;
            global_best.best_position = particles[i].best_position;
        }
    }

    int iter_n = 0;

    for (unsigned int i = 0; i < iters_num; ++i) {
        omp_set_num_threads(omp_get_max_threads());
#pragma omp parallel for
        for (unsigned int j = 0; j < PSO_POPULATION_SIZE; ++j) {
            for (unsigned int k = 0; k < 3; ++k) {
                Array2d random_coef = Array2d::Random().abs();
                particles[j].velocity[k] = w * particles[j].velocity[k]
                    + c1 * random_coef[0] * (particles[j].best_position[k] - particles[j].position[k])
                    + c2 * random_coef[1] * (global_best.best_position[k] - particles[j].position[k]);

                particles[j].position[k] = particles[j].position[k] + particles[j].velocity[k];
            }

            particles[j].cost = cost_function(particles[j].position, ref_frame, new_frame);

            if (particles[j].cost < particles[j].best_cost) {
                particles[j].best_cost = particles[j].cost;
                particles[j].best_position = particles[j].position;
            }
#pragma omp critical
            if (particles[j].cost < global_best.best_cost) {
                iter_n = i;
                global_best.best_cost = particles[j].best_cost;
                global_best.best_position = particles[j].best_position;
            }
        }

        w *= w_damping_coef;
    }

    printf("Global Best Cost (PSO): %03.5f \tIteration: %d\n"
           "Pose (x, y, theta): (%04.5f, %04.5f, %02.5f)\n",
        global_best.best_cost,
        iter_n,
        global_best.best_position[0],
        global_best.best_position[1],
        global_best.best_position[2]);
    return global_best.best_position;
}

Vector3d glir_pso_optimization(Vector3d initial_guess, NDTFrame* const ref_frame, NDTFrame* const new_frame, unsigned int iters_num, Array3d deviation)
{
    double omega = 1., c1 = 2., c2 = 2.;
    Array3d zero_devi;
    zero_devi << 1E-4, 1E-4, 1E-5;

    Particle global_best(initial_guess, deviation, ref_frame, new_frame);

    vector<Particle> particles;
    int iter_n = 0;

    particles.push_back(Particle(initial_guess.array(), deviation, ref_frame, new_frame));

    for (unsigned int i = 0; i < PSO_POPULATION_SIZE; ++i) {
        particles.push_back(Particle(initial_guess.array(), deviation, ref_frame, new_frame));

        if (particles[i].cost < global_best.best_cost) {
            global_best.best_cost = particles[i].best_cost;
            global_best.best_position = particles[i].best_position;
        }
    }

    for (unsigned int i = 0; i < iters_num; ++i) {
        double pbest_avr = .0;
        for (unsigned int j = 0; j < PSO_POPULATION_SIZE; ++j) {
            omega = 1.1 - global_best.best_cost / (particles[j].pbest_average / (j + 1));
            c1 = c2 = 1.0 + global_best.best_cost / particles[j].best_cost;
            for (unsigned int k = 0; k < 3; ++k) {
                Array2d random_coef = Array2d::Random().abs();
                double best_ratio = particles[j].best_position[k] / global_best.best_position[k];
                particles[j].velocity[k] = omega * particles[j].velocity[k]
                    + c1 * random_coef[0] * (best_ratio * particles[j].best_position[k] - particles[j].position[k])
                    + c2 * random_coef[1] * ((1. / best_ratio) * global_best.best_position[k] - particles[j].position[k]);

                particles[j].position[k] = particles[j].position[k] + particles[j].velocity[k];
            }

            particles[j].cost = cost_function(particles[j].position, ref_frame, new_frame);

            if (particles[j].cost < particles[j].best_cost) {
                particles[j].best_cost = particles[j].cost;
                particles[j].best_position = particles[j].position;
            }

            pbest_avr += particles[j].best_cost;
            particles[j].pbest_average += particles[j].best_cost;

            if (particles[j].cost < global_best.best_cost) {
                iter_n = i;
                global_best.best_cost = particles[j].best_cost;
                global_best.best_position = particles[j].best_position;
            }
        }
        //        omega = 1.1 - global_best.best_cost / (particles[j].pbest_average / (i + 1));
    }

    printf("Global Best Cost (PSO): %03.5f \tIteration: %d\n"
           "Pose (x, y, theta): (%04.5f, %04.5f, %02.5f)\n",
        global_best.best_cost,
        iter_n,
        global_best.best_position[0],
        global_best.best_position[1],
        global_best.best_position[2]);
    return global_best.best_position;
}
