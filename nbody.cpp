#include <vector>
#include <cmath>
#include <climits>
#include <thread>
#include <numeric>
#include <iostream>
#include <iterator>
#include <optional>
#include <vector>
#include <chrono>
#include <atomic>

#include "nbody.hpp"

NBodySimulation::NBodySimulation(double gravitationalConstant, double timeStep) : G(gravitationalConstant), dt(timeStep) {}

void NBodySimulation::addBody(double mass, double x, double y, double vx, double vy, double r, double g, double b) {
    Body body;
    body.mass = mass;
    body.x = x;
    body.y = y;
    body.vx = vx;
    body.vy = vy;
    body.fx = 0.0;
    body.fy = 0.0;
    body.r = r;
    body.g = g;
    body.b = b;
    
    bodies.push_back(body);
}

void NBodySimulation::clear() {
    bodies.clear();
}

void NBodySimulation::setTimeStep(double new_dt) {
    dt = new_dt;
}

// Compute forces between bodies
void NBodySimulation::computeForces() {
    // Reset forces
    for (auto& body : bodies) {
        body.fx = 0.0;
        body.fy = 0.0;
    }
    // Compute forces between each pair of bodies
    for (size_t i = 0; i < bodies.size(); ++i) {
        for (size_t j = i+1; j < bodies.size(); ++j) {
            // squared distances 
            double dx = bodies[j].x - bodies[i].x;
            double dy = bodies[j].y - bodies[i].y;
                
            double distSquared = dx*dx + dy*dy;
            if (distSquared == 0) {
                distSquared = 1e-10;
            }
                
            // force magnitude
            double force = G * bodies[i].mass * bodies[j].mass / distSquared;
                
            // force vectors
            double dist = sqrt(distSquared);
            double fx = force * dx / dist;
            double fy = force * dy / dist;
            // Newton's Third law
            bodies[i].fx += fx;
            bodies[i].fy += fy;
            bodies[j].fx -= fx;
            bodies[j].fy -= fy;
        }
    }
}

void NBodySimulation::ComputeForcesThreadAtomic(
    Body* bodies,
    std::vector<std::atomic<double>>& fx_arr,
    std::vector<std::atomic<double>>& fy_arr,
    size_t start,
    size_t end,
    double G,
    size_t total_bodies
) {
    std::vector<double> local_fx(total_bodies, 0.0);
    std::vector<double> local_fy(total_bodies, 0.0);

    for (size_t i = start; i < end; ++i) { 
        for (size_t j = i + 1; j < total_bodies; ++j) { 
            double dx = bodies[j].x - bodies[i].x;
            double dy = bodies[j].y - bodies[i].y;
            double distSquared = dx * dx + dy * dy;
            if (distSquared == 0.0) {
                distSquared = 1e-10;
            }

            double dist = std::sqrt(distSquared);
            double force = G * bodies[i].mass * bodies[j].mass / distSquared;
            double fx = force * dx / dist;
            double fy = force * dy / dist;

            local_fx[i] += fx;
            local_fy[i] += fy;
            local_fx[j] -= fx;
            local_fy[j] -= fy;
        }
    }

    for (size_t i = 0; i < total_bodies; ++i) {
        if (local_fx[i] != 0.0) {
            fx_arr[i].fetch_add(local_fx[i]);
        }
        if (local_fy[i] != 0.0) {
            fy_arr[i].fetch_add(local_fy[i]);
        }
    }
}


void NBodySimulation::computeForcesParallelAtomic(size_t num_threads) {
    size_t N = bodies.size();
    if (N == 0){
        return; 
    }

    std::vector<std::atomic<double>> fx_arr(N);
    std::vector<std::atomic<double>> fy_arr(N);

    for (size_t i = 0; i < N; ++i) { 
        fx_arr[i] = 0.0;
        fy_arr[i] = 0.0;
    }

    size_t block_size = (N + num_threads - 1) / num_threads;

    if (block_size == 0){
        block_size = 1;
    }
    std::vector<std::thread> workers(num_threads - 1);

    size_t start_block = 0;
    for (size_t i = 0; i < num_threads-1; ++i) {
        size_t end_block = std::min(start_block + block_size, N);

        workers[i] = std::thread(&NBodySimulation::ComputeForcesThreadAtomic,
                                this,
                                bodies.data(),
                                std::ref(fx_arr),
                                std::ref(fy_arr),
                                start_block,
                                end_block,
                                G,
                                N);
        start_block = end_block;
    }

    ComputeForcesThreadAtomic(bodies.data(), std::ref(fx_arr), std::ref(fy_arr), start_block, N, G, N);
 
    for (size_t i = 0; i < workers.size(); ++i) {
        workers[i].join();
    }

    for (size_t i = 0; i < N; ++i) {
        bodies[i].fx = fx_arr[i];
        bodies[i].fy = fy_arr[i];
    }
}


void NBodySimulation::ComputeForcesThreadNonAtomic(
    Body* bodies,
    std::vector<std::vector<double>>& local_fx,
    std::vector<std::vector<double>>& local_fy,
    size_t start,
    size_t end,
    double G,
    size_t total_bodies,
    int thread_id
) {
    for (size_t i = start; i < end; ++i) {
        for (size_t j = i + 1; j < total_bodies; ++j) {
            double dx = bodies[j].x - bodies[i].x;
            double dy = bodies[j].y - bodies[i].y;
            double distSquared = dx * dx + dy * dy;
            if (distSquared == 0.0) {
                distSquared = 1e-10;
            }

            double dist = std::sqrt(distSquared);
            double force = G * bodies[i].mass * bodies[j].mass / distSquared;
            double fx = force * dx / dist;
            double fy = force * dy / dist;

            local_fx[thread_id][i] += fx;
            local_fy[thread_id][i] += fy;
            local_fx[thread_id][j] -= fx;
            local_fy[thread_id][j] -= fy;
        }
    }
}

void NBodySimulation::computeForcesParallelNonAtomic(size_t num_threads) {
    size_t N = bodies.size();
    if (N == 0){
        return; 
    }

    // thread-local arrays
    std::vector<std::vector<double>> local_fx(num_threads, std::vector<double>(N, 0.0));
    std::vector<std::vector<double>> local_fy(num_threads, std::vector<double>(N, 0.0));

    size_t block_size = (N + num_threads - 1) / num_threads;

    if (block_size == 0){
        block_size = 1;
    }
    std::vector<std::thread> workers(num_threads - 1);
    size_t start_block = 0;
    for (size_t i = 0;i < num_threads-1; ++i) {
        size_t end_block = std::min(start_block + block_size, N);
        workers[i] = std::thread(&NBodySimulation::ComputeForcesThreadNonAtomic,
                                this,
                                bodies.data(),
                                std::ref(local_fx),
                                std::ref(local_fy),
                                start_block,
                                end_block,
                                G,
                                N,
                                i);

        start_block = end_block;
    }

    ComputeForcesThreadNonAtomic(bodies.data(), std::ref(local_fx), std::ref(local_fy), start_block, N, G, N, num_threads-1);
    
    for (size_t i = 0; i < workers.size(); ++i) {
        workers[i].join();
    }

    // Combine thread-local forces into final result
    for (size_t i = 0; i < N; ++i) {
        double fx = 0.0;
        double fy = 0.0;
        for (size_t t = 0; t < num_threads; ++t) {
            fx += local_fx[t][i];
            fy += local_fy[t][i];
        }
        bodies[i].fx = fx;
        bodies[i].fy = fy;
    }
}


// update components 
void NBodySimulation::updatePositionsSeq() {
    for (auto& body : bodies) {
        // acceleration
        double ax = body.fx / body.mass;
        double ay = body.fy / body.mass;
        // velocity
        body.vx += ax * dt;
        body.vy += ay * dt;
        // position
        body.x += body.vx * dt;
        body.y += body.vy * dt;
    }
}

void NBodySimulation::updatePositionsThread(size_t start, size_t end, double dt, std::vector<Body>& bodies) {
    for (size_t i = start; i < end; ++i) {
        double ax = bodies[i].fx / bodies[i].mass;
        double ay = bodies[i].fy / bodies[i].mass;
        bodies[i].vx += ax * dt;
        bodies[i].vy += ay * dt;
        bodies[i].x += bodies[i].vx * dt;
        bodies[i].y += bodies[i].vy * dt;
    }
}

void NBodySimulation::updatePositionsParallel(size_t num_threads) {
    size_t length = bodies.size();
    if (length == 0) {
        return;
    }

    size_t block_size = length / num_threads;
    if (block_size == 0){
        block_size = 1;
    }

    std::vector<std::thread> workers(num_threads - 1);

    size_t start_block = 0;
    for (size_t i = 0; i < num_threads - 1; ++i) {
        size_t end_block = start_block + block_size;
        workers[i] = std::thread(&NBodySimulation::updatePositionsThread, start_block, end_block, dt, std::ref(bodies));
        start_block = end_block;
    }
    updatePositionsThread(start_block, length, dt, bodies);

    for (size_t i = 0; i < workers.size(); ++i) {
        workers[i].join();
    }
}

// one step of the simulation
void NBodySimulation::stepSequential() {
    computeForces();
    updatePositionsSeq();
}

void NBodySimulation::stepParallel(size_t num_threads) {
    computeForcesParallelAtomic(num_threads);
    updatePositionsParallel(num_threads);
}

const std::vector<Body>& NBodySimulation::getBodies() const {
    return bodies;
}


const double G = 6.67430e-11;
double dt = 10000.0;

// Compute the position relative error to test the accuracy of the parallel approach
void PositionRelativeError(const std::vector<Body>& a, const std::vector<Body>& b, double& error_out) {
    if (a.size() != b.size()) {
        error_out = std::numeric_limits<double>::infinity();
        return;
    }

    double total_error = 0.0;
    const double epsilon = 1e-12;

    for (size_t i = 0; i < a.size(); ++i) {
        double dx = a[i].x - b[i].x;
        double dy = a[i].y - b[i].y;

        double dist = std::sqrt(dx * dx + dy * dy);
        double seq_position = std::sqrt(a[i].x * a[i].x + a[i].y * a[i].y);

        total_error += dist / std::max(seq_position, epsilon);
    }

    error_out = total_error / a.size();
}
