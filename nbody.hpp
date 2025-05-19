#ifndef NBODY_HPP
#define NBODY_HPP

#include <vector>
#include <thread>
#include <mutex>

// Structure representing a celestial body
struct Body {
    double mass;
    double x, y;
    double vx, vy;
    double fx, fy;
    double r, g, b;
};

class NBodySimulation {
private:
    std::vector<Body> bodies;
    double G;   // Gravitational constant
    double dt;  // Time step


public:
    NBodySimulation(double gravitationalConstant, double timeStep);

    void addBody(double mass, double x, double y, double vx, double vy, double r = 1.0, double g = 1.0, double b = 1.0);

    void clear();
    
    void setTimeStep(double new_dt);

    void computeForces();

    void ComputeForcesThread(
        Body* bodies,
        std::vector<double>& fx_arr,
        std::vector<double>& fy_arr,
        std::vector<std::mutex>& body_mutexes,
        size_t start,
        size_t end,
        double G,
        size_t total_bodies
    );

    void computeForcesParallel(size_t num_threads);


    void updatePositionsSeq();

    static void updatePositionsThread(size_t start, size_t end, double dt, std::vector<Body>& bodies);

    void updatePositionsParallel(size_t num_threads);

    void stepSequential();
    void stepParallel(size_t num_threads);

    const std::vector<Body>& getBodies() const;
};

// Constants
extern const double G;
extern double dt;
extern const double epsilon;

// Body comparison for testing
bool compareBodies(const std::vector<Body>& a, const std::vector<Body>& b);

#endif // NBODY_HPP
