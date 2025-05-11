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

// Thread function for computing forces
void NBodySimulation::ComputeForcesThread(Body* bodies,
                                          std::vector<double>& fx_arr,
                                          std::vector<double>& fy_arr,
                                          std::mutex& mtx,
                                          size_t start,
                                          size_t end,
                                          double G,
                                          size_t total_bodies) {
                                            
    for (size_t i = start; i < end && i < total_bodies; ++i) {
        double fx_local = 0.0;
        double fy_local = 0.0;
        for (size_t j = i + 1; j < total_bodies; ++j) {
            double dx = bodies[j].x - bodies[i].x;
            double dy = bodies[j].y - bodies[i].y;
            double distSquared = dx * dx + dy * dy;
            if (distSquared == 0.0){
                distSquared = 1e-10;
            }
            double dist = std::sqrt(distSquared);
            double force = G * bodies[i].mass * bodies[j].mass / distSquared;
            double fx = force * dx / dist;
            double fy = force * dy / dist;

            fx_local += fx;
            fy_local += fy;

            {
                std::lock_guard<std::mutex> lock(mtx);
                fx_arr[j] -= fx;
                fy_arr[j] -= fy;
            }
        }

        {
            std::lock_guard<std::mutex> lock(mtx);
            fx_arr[i] += fx_local;
            fy_arr[i] += fy_local;
        }
    }
}

void NBodySimulation::computeForcesParallel(size_t num_threads) {
    size_t N = bodies.size();
    if (N == 0){
        return; 
    }

    std::vector<double> fx_arr(N, 0.0);
    std::vector<double> fy_arr(N, 0.0);
    std::mutex mtx;

    size_t block_size = N / num_threads;
    if (block_size == 0){
        block_size = 1;
    }
    std::vector<std::thread> workers(num_threads - 1);

    size_t start_block = 0;
    for (size_t i = 0; i < num_threads - 1; ++i) {
        size_t end_block = start_block + block_size;
        workers[i] = std::thread(&NBodySimulation::ComputeForcesThread,
                                bodies.data(),
                                std::ref(fx_arr),
                                std::ref(fy_arr),
                                std::ref(mtx),
                                start_block,
                                end_block,
                                G,
                                N);
        start_block = end_block;
    }


    ComputeForcesThread(bodies.data(), std::ref(fx_arr), std::ref(fy_arr), std::ref(mtx), start_block, N, G, N);

    for (size_t i = 0; i < workers.size(); ++i) {
        workers[i].join();
    }

    for (size_t i = 0; i < N; ++i) {
        bodies[i].fx = fx_arr[i];
        bodies[i].fy = fy_arr[i];
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
    computeForcesParallel(num_threads);
    updatePositionsParallel(num_threads);
}

const std::vector<Body>& NBodySimulation::getBodies() const {
    return bodies;
}


const double G = 6.67430e-11;
double dt = 10000.0;

bool compareBodies(const std::vector<Body>& a, const std::vector<Body>& b) {
    const double epsilon = 1e-6;
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) { 
        if (std::abs(a[i].x - b[i].x) > epsilon ||
            std::abs(a[i].y - b[i].y) > epsilon ||
            std::abs(a[i].vx - b[i].vx) > epsilon ||
            std::abs(a[i].vy - b[i].vy) > epsilon) {
                
            std::cout << "Bodies differ at index " << i << std::endl;
            std::cout << "Diff x " << std::abs(a[i].x - b[i].x) << std::endl;
            std::cout << "Diff y " << std::abs(a[i].y - b[i].y) << std::endl;
            std::cout << "Diff vx " << std::abs(a[i].vx - b[i].vx) << std::endl;
            std::cout << "Diff vy " << std::abs(a[i].vy - b[i].vy) << std::endl;

            return false;
        }
    }
    return true;
}
