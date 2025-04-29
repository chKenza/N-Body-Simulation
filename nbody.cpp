#include <iostream>
#include <vector>
#include <cmath>

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
    void updatePositions() {
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

    // one step of the simulation
    void step() {
        computeForces();
        updatePositions();
    }

    const std::vector<Body>& getBodies() const {
        return bodies;
    }
};

const double G = 6.67430e-11;
const double dt = 10000;

int main() {
    // create simulation
    NBodySimulation sim(G, dt);
    // Add bodies
    // center body (similar to the sun)
    sim.addBody(1.989e30, 0.0, 0.0, 0.0, 0.0);
    
    // similar to earth
    sim.addBody(5.972e24, 1.49e11, 0.0, 0.0, 29800.0);
    
    // similar to mars
    sim.addBody(6.39e23, 2.28e11, 0.0, 0.0, 24100.0);
    
    // Simulate
    for (int step = 0; step <= 1000; ++step) {
        sim.step();
        
        if (step % 100 == 0) {
            std::cout << "Step " << step << ":\n";
            const auto& bodies = sim.getBodies();
            for (size_t i = 0; i < bodies.size(); ++i) {
                std::cout << "Body " << i << ": ("
                          << bodies[i].x << ", " << bodies[i].y << ")\n";
            }
            std::cout << "\n";
        }
    }
    
    return 0;
}