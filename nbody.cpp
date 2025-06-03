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

void NBodySimulation::addBody(Body body) {
    body.fx = 0.0;
    body.fy = 0.0;
    bodies.push_back(body);
}

void NBodySimulation::clear() {
    bodies.clear();
}

void NBodySimulation::setTimeStep(double new_dt) {
    dt = new_dt;
}

// Compute forces between bodies with the sequential approach
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

// Compute forces between bodies with the parallel approach using atomics
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

// Compute forces between bodies with the parallel approach using thread-local arrays
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

void NBodySimulation::stepParallel(size_t num_threads, double theta) {
    // change to computeForcesParallelAtomic(num_threads) to use the atomic parallel approach
    // change to computeForcesParallelNonAtomic(num_threads) to use the non atomic parallel approach
    // change to ComputeForcesParallelBarnesHut(size_t num_threads, double theta) to use the Barnes Hut algorithm
    ComputeForcesParallelBarnesHut(num_threads, theta);
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



// Barnes-Hut Algorithm implementation
// ChatGpt helped with the implementation of this algorithm
/*
We devide the space as follows:
Divide space recursively
+--------+--------+
|        |        |
|   NW   |   NE   |
|        |        |
+--------+--------+
|        |        |
|   SW   |   SE   |
|        |        |
+--------+--------+
approximate if body far enough, i.e. size of region / dist < theta
*/

Quad::Quad(double xmid, double ymid, double length)
    : xmid(xmid), ymid(ymid), length(length) {}

bool Quad::contains(double x, double y) const {
    // check if (x, y) is in the square
    return x >= xmid - length / 2 && x <= xmid + length / 2 &&
           y >= ymid - length / 2 && y <= ymid + length / 2;
}

Quad Quad::NW() const { return Quad(xmid - length / 4, ymid + length / 4, length / 2); }
Quad Quad::NE() const { return Quad(xmid + length / 4, ymid + length / 4, length / 2); }
Quad Quad::SW() const { return Quad(xmid - length / 4, ymid - length / 4, length / 2); }
Quad Quad::SE() const { return Quad(xmid + length / 4, ymid - length / 4, length / 2); }


QuadNode::QuadNode(const Quad& region) : region(region) {}

bool QuadNode::isExternal() const {
    // True if node has no children
    return !NW && !NE && !SW && !SE;
}

void QuadNode::insert(Body* b) {
    // No insertion if body is outside this node's region
    if (!region.contains(b->x, b->y)) return;

    // First body to be inserted
    if (!body && isExternal()) {
        // store and set center of mass
        body = b;
        mass = b->mass;
        com_x = b->x;
        com_y = b->y;
        return;
    }

    // If the node already contains a body, we subdivide
    if (isExternal()) {
        Body* existing = body;
        body = nullptr;
        // Create the four child quadrants
        NW = std::make_unique<QuadNode>(region.NW());
        NE = std::make_unique<QuadNode>(region.NE());
        SW = std::make_unique<QuadNode>(region.SW());
        SE = std::make_unique<QuadNode>(region.SE());

        // Reinsert the existing body into one of the children
        if (NW->region.contains(existing->x, existing->y)) NW->insert(existing);
        else if (NE->region.contains(existing->x, existing->y)) NE->insert(existing);
        else if (SW->region.contains(existing->x, existing->y)) SW->insert(existing);
        else if (SE->region.contains(existing->x, existing->y)) SE->insert(existing);
    }

    // Insert the new body
    if (NW->region.contains(b->x, b->y)) NW->insert(b);
    else if (NE->region.contains(b->x, b->y)) NE->insert(b);
    else if (SW->region.contains(b->x, b->y)) SW->insert(b);
    else if (SE->region.contains(b->x, b->y)) SE->insert(b);

    // Update center of mass
    double new_mass = mass + b->mass;
    com_x = (com_x * mass + b->x * b->mass) / new_mass;
    com_y = (com_y * mass + b->y * b->mass) / new_mass;
    mass = new_mass;
}


void NBodySimulation::ComputeForceBarnesHut(Body& b, const QuadNode& node, double G, double theta) {
    if (node.mass == 0 || (node.isExternal() && node.body == &b)) return;

    double dx = node.com_x - b.x;
    double dy = node.com_y - b.y;
    double distSquared = dx * dx + dy * dy;
    if (distSquared == 0.0) {
        distSquared = 1e-10;
    }

    double dist = std::sqrt(distSquared);

    // If size of region / dist < theta then we approximate (far enough)
    if (node.isExternal() || (node.region.length / dist < theta)) {
        double F = G * b.mass * node.mass / (dist * dist);
        b.fx += F * dx / dist;
        b.fy += F * dy / dist;
    } else {
        // too close, recurse
        if (node.NW) ComputeForceBarnesHut(b, *node.NW, G, theta);
        if (node.NE) ComputeForceBarnesHut(b, *node.NE, G, theta);
        if (node.SW) ComputeForceBarnesHut(b, *node.SW, G, theta);
        if (node.SE) ComputeForceBarnesHut(b, *node.SE, G, theta);
    }
}
void NBodySimulation::ComputeForcesThreadBarnesHut(
    Body* bodies,
    size_t start,
    size_t end,
    const QuadNode* root,
    double G,
    double theta
) {
    for (size_t i = start; i < end; ++i) {
        bodies[i].fx = 0.0;
        bodies[i].fy = 0.0;
        ComputeForceBarnesHut(bodies[i], *root, G, theta);
    }
}

void NBodySimulation::ComputeForcesParallelBarnesHut(size_t num_threads, double theta) {
    size_t N = bodies.size();
    if (N == 0){
        return; 
    }

    size_t block_size = (N + num_threads - 1) / num_threads;

    if (block_size == 0){
        block_size = 1;
    }

    // Compute bounds to enclose all bodies
    double min_x = bodies[0].x, max_x = bodies[0].x;
    double min_y = bodies[0].y, max_y = bodies[0].y;
    for (const auto& b : bodies) {
        min_x = std::min(min_x, b.x);
        max_x = std::max(max_x, b.x);
        min_y = std::min(min_y, b.y);
        max_y = std::max(max_y, b.y);
    }

    // Define the root quad to cover all bodies
    double length = std::max(max_x - min_x, max_y - min_y);
    Quad root_region((min_x + max_x) / 2.0, (min_y + max_y) / 2.0, length * 1.5);
    QuadNode root(root_region);

    //Build tree
    for (auto& b : bodies) {
        root.insert(&b);
    }

    // double theta = 0.5; // Make smaller to increase accuracy, make bigger to increase speedup
    std::vector<std::thread> workers(num_threads - 1);
    size_t start_block = 0;
    for (size_t i = 0; i < num_threads -1; ++i) {
        size_t end_block = std::min(start_block + block_size, N);
        workers[i] = std::thread(&NBodySimulation::ComputeForcesThreadBarnesHut,
                                    this,
                                    bodies.data(),
                                    start_block,
                                    end_block,
                                    &root,
                                    G,
                                    theta);

        start_block = end_block;
    }

    ComputeForcesThreadBarnesHut(bodies.data(), start_block, N, &root, G, theta);

    for (size_t i = 0; i < workers.size(); ++i) {
        workers[i].join();
    }
    

}
