#ifndef NBODY_HPP
#define NBODY_HPP

#include <vector>
#include <thread>
#include <mutex>
#include <memory>

// Structure representing a celestial body
struct Body {
    double mass;
    double x, y;
    double vx, vy;
    double fx, fy;
    double r, g, b;
};

// Structure for Barnes-Hut Algorithm
struct Quad {
    double xmid, ymid, length;  // center and length of the square
    Quad(double xmid, double ymid, double length);
    // to check is the point is in the region
    bool contains(double x, double y) const;

    //new Quad
    Quad NW() const;
    Quad NE() const;
    Quad SW() const;
    Quad SE() const;
};

struct QuadNode {
    Quad region;
    double mass = 0;
    double com_x = 0, com_y = 0; // center of mass
    Body* body = nullptr;
    std::unique_ptr<QuadNode> NW, NE, SW, SE;

    QuadNode(const Quad& region);
    bool isExternal() const;
    void insert(Body* b);  // insert body into the quadtree
};

class NBodySimulation {
private:
    std::vector<Body> bodies;
    double G;   // Gravitational constant
    double dt;  // Time step


public:
    NBodySimulation(double gravitationalConstant, double timeStep);

    void addBody(double mass, double x, double y, double vx, double vy, double r = 1.0, double g = 1.0, double b = 1.0);

    void addBody(Body body);

    void clear();
    
    void setTimeStep(double new_dt);

    void computeForces();

    void ComputeForcesThreadAtomic(
        Body* bodies,
        std::vector<std::atomic<double>>& fx_arr,
        std::vector<std::atomic<double>>& fy_arr,
        size_t start,
        size_t end,
        double G,
        size_t total_bodies
    );

    void computeForcesParallelAtomic(size_t num_threads);

    void ComputeForcesThreadNonAtomic(
        Body* bodies,
        std::vector<std::vector<double>>& local_fx,
        std::vector<std::vector<double>>& local_fy,
        size_t start,
        size_t end,
        double G,
        size_t total_bodies,
        int thread_id
    );

    void computeForcesParallelNonAtomic(size_t num_threads);


    void updatePositionsSeq();

    static void updatePositionsThread(size_t start, size_t end, double dt, std::vector<Body>& bodies);

    void updatePositionsParallel(size_t num_threads);

    void stepSequential();

    void stepParallel(size_t num_threads, double theta = 0.5); // Barnes-Hut parameter

    const std::vector<Body>& getBodies() const;

    void ComputeForceBarnesHut(Body& b, const QuadNode& node, double G, double theta);
    void ComputeForcesParallelBarnesHut(size_t num_threads, double theta);
    void ComputeForcesThreadBarnesHut(
        Body* bodies,
        size_t start,
        size_t end,
        const QuadNode* root,
        double G,
        double theta
    );

};

// Constants
extern const double G;
extern double dt;

// Relative error for testing
void PositionRelativeError(const std::vector<Body>& a, const std::vector<Body>& b, double& error_out);


#endif // NBODY_HPP
