#include <iostream>
#include <vector>
#include <cmath>
#include <climits>
#include <thread>
#include <numeric>
#include <iterator>
#include <optional>
#include <vector>
#include <chrono>

// Structure of the body: mass, coordinates, velocity, force
struct Body {
    double mass;
    double x, y;
    double vx, vy;
    double fx, fy;
};

class NBodySimulation {
private:
    std::vector<Body> bodies;
    double G;  // gravitational cte
    double dt; // delta t

public:
    NBodySimulation(double gravitationalConstant, double timeStep) : G(gravitationalConstant), dt(timeStep) {}

    // Add a body
    void addBody(double mass, double x, double y, double vx, double vy) {
        Body body;
        body.mass = mass;
        body.x = x;
        body.y = y;
        body.vx = vx;
        body.vy = vy;
        body.fx = 0.0;
        body.fy = 0.0;
        
        bodies.push_back(body);
    }

    // Compute forces between bodies
    void computeForces() {
        // Reset forces
        for (auto& body : bodies) {
            body.fx = 0.0;
            body.fy = 0.0;
        }

        // Compute forces between each pair of bodies
        for (size_t i = 0; i < bodies.size(); ++i) {
            for (size_t j = 0; j < bodies.size(); ++j) {
                if (i != j) {
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

                    bodies[i].fx += fx;
                    bodies[i].fy += fy;
                }
            }
        }
    }

    // update components 
    void updatePositionsSeq() {
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

    static void updatePositionsThread(int start, int end, double dt, std::vector<Body>& bodies) {
        for (int i = start; i < end; ++i) {
            double ax = bodies[i].fx / bodies[i].mass;
            double ay = bodies[i].fy / bodies[i].mass;
            bodies[i].vx += ax * dt;
            bodies[i].vy += ay * dt;
            bodies[i].x += bodies[i].vx * dt;
            bodies[i].y += bodies[i].vy * dt;
        }
    }
    
    void updatePositionsParallel(size_t num_threads) {
        size_t length = bodies.size();
        if (length == 0) {
            return;
        }
    
        size_t block_size = length / num_threads;
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

    void stepSequential() {
        computeForces();
        updatePositionsSeq();
    }
    void stepParallel(size_t num_threads) {
        computeForces();
        updatePositionsParallel(num_threads);
    }

    const std::vector<Body>& getBodies() const {
        return bodies;
    }
};

const double G = 6.67430e-11;
const double dt = 10000;
const double epsilon = 1e-6;

bool compareBodies(const std::vector<Body>& a, const std::vector<Body>& b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        if (std::abs(a[i].x - b[i].x) > epsilon ||
            std::abs(a[i].y - b[i].y) > epsilon ||
            std::abs(a[i].vx - b[i].vx) > epsilon ||
            std::abs(a[i].vy - b[i].vy) > epsilon) {
            return false;
        }
    }
    return true;
}

int main() {
    // create simulation
    NBodySimulation sim_seq(G, dt);
    NBodySimulation sim_par(G, dt);

    // Add bodies
    // center body (similar to the sun)
    sim_seq.addBody(1.989e30, 0.0, 0.0, 0.0, 0.0); 
    // similar to earth
    sim_seq.addBody(5.972e24, 1.49e11, 0.0, 0.0, 29800.0);
    // similar to mars
    sim_seq.addBody(6.39e23, 2.28e11, 0.0, 0.0, 24100.0);
    
    sim_par.addBody(1.989e30, 0.0, 0.0, 0.0, 0.0);
    sim_par.addBody(5.972e24, 1.49e11, 0.0, 0.0, 29800.0);
    sim_par.addBody(6.39e23, 2.28e11, 0.0, 0.0, 24100.0);

    const size_t num_threads = 4;


    // simulate

    for (int step = 0; step <= 1000; ++step) {
        sim_seq.stepSequential();
        sim_par.stepParallel(num_threads);

        if (step % 100 == 0) {
            std::cout << "Step " << step << ":\n";
            const auto& bodies_seq = sim_seq.getBodies();
            const auto& bodies_par = sim_par.getBodies();
            for (size_t i = 0; i < bodies_seq.size(); ++i) {
                std::cout << "Body " << i << " (Seq): (" << bodies_seq[i].x << ", " << bodies_seq[i].y << ")\n";
                std::cout << "Body " << i << " (Par): (" << bodies_par[i].x << ", " << bodies_par[i].y << ")\n";
            }
            //bool same = compareBodies(bodies_seq, bodies_par);
            //std::cout << (same) << std::endl;
        }
    }
    
    return 0;
}
