#include "ndtpso_slam/core.h"
#include "ndtpso_slam/ndtframe.h"
#include <iostream>

using std::cout;

struct Particle {
    Vector3d position, velocity, best_position;
    double best_cost;
    double cost;

    Particle(Array3d mean, Array3d deviation, NdtFrame* const ref_frame, NdtFrame* const new_frame)
    {
        position = mean + ((Array3d::Random() * deviation)); // Randomly place the particles according to mean and deviation
        velocity << 0, 0, 0;

        cost = cost_function(position, ref_frame, new_frame);

        best_position = position;
        best_cost = cost;
    }
};

Vector3d pso_optimization(Vector3d initial_guess, NdtFrame* const ref_frame, NdtFrame* const new_frame, unsigned int iters_num)
{
    iters_num = 100;
    //    def pso(mean, ref_frame, new_frame, iters=25):
    double w = 1., w_damping_coef = .4, c1 = 2., c2 = 2.;
    unsigned short num_of_particles = 70;
    Array3d deviation;
    deviation << .5, .2, M_PI / 10.;

    //    Particle global_best(initial_guess, deviation, ref_frame, new_frame);

    //    vector<Particle> particles(num_of_particles - 1, Particle(initial_guess.array(), deviation, ref_frame, new_frame));
    vector<Particle> particles;
    //    particles.push_back(global_best);
    unsigned int global_best_index = 0;

    for (unsigned int i = 0; i < num_of_particles; ++i) {
        particles.push_back(Particle(initial_guess.array(), deviation, ref_frame, new_frame));

        if (particles[i].cost < particles[global_best_index].cost) {
            global_best_index = i;
        }
    }

    for (unsigned int i = 0; i < iters_num; ++i) {
        for (unsigned int j = 0; j < num_of_particles; ++j) {
            for (unsigned int k = 0; k < 3; ++k) {
                Array2d random_coef = Array2d::Random().abs();
                particles[j].velocity[k] = w * particles[j].velocity[k]
                    + c1 * random_coef[0] * (particles[j].best_position[k] - particles[j].position[k])
                    + c2 * random_coef[1] * (particles[global_best_index].position[k] - particles[j].position[k]);

                particles[j].position[k] = particles[j].position[k] + particles[j].velocity[k];
            }

            particles[j].cost = cost_function(particles[j].position, ref_frame, new_frame);

            if (particles[j].cost < particles[j].best_cost) {
                particles[j].best_cost = particles[j].cost;
                particles[j].best_position = particles[j].position;
            }

            if (particles[j].cost < particles[global_best_index].cost) {
                global_best_index = j;
            }
        }

        w *= w_damping_coef;
    }

    //    cout << "Blobal Best Cost ";
    //    cout << particles[global_best_index].best_cost;
    //    cout << std::endl;
    return particles[global_best_index].best_position;
}
