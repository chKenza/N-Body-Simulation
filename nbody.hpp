#ifndef NBODY_HPP
#define NBODY_HPP

#include <vector>
#include <thread>

// Structure representing a celestial body
struct Body {
    double mass;
    double x, y;
    double vx, vy;
    double fx, fy;
};

class NBodySimulation {
private:
    std::vector<Body> bodies;
    double G;   // Gravitational constant
    double dt;  // Time step

public:
    NBodySimulation(double gravitationalConstant, double timeStep);

    void addBody(double mass, double x, double y, double vx, double vy);

    void computeForces();

    void updatePositionsSeq();

    static void updatePositionsThread(int start, int end, double dt, std::vector<Body>& bodies);

    void updatePositionsParallel(size_t num_threads);

    void stepSequential();
    void stepParallel(size_t num_threads);

    const std::vector<Body>& getBodies() const;
};

// Constants
extern const double G;
extern const double dt;
extern const double epsilon;

// Body comparison for testing
bool compareBodies(const std::vector<Body>& a, const std::vector<Body>& b);

#endif // NBODY_HPP
