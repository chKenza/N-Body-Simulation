# N-Body Simulation
 Simulating the dynamics of several bodies moving simultaneously under the influence of gravity forces in 2D. It includes both sequential and parallel implementations, along with a real-time GTKMM GUI for visualization.

## Files Overview

- **nbody.hpp** : Declares the Body struct and the NBodySimulation class.
- **nbody.cpp** : Implements the time-step update and the force computations in sequential and parallel approaches (e.g., a direct parallel approach using atomics; a direct parallel approach using thread-local arrays; and the parallel Barnes-Hut algorithm).
- **sim.cpp** : Starts the GUI with GTKMM and allows switching between simulation modes (solar system, 3-body and 6-body problems, N random bodies, etc.). It also implements the accuracy and efficiency analyses.

## How to Run
### 1. Prerequisites
Make sure you have:
- A **C++20**-compatible compiler
- **GTKMM 3** library
### 2. Build the Project
Download all the source files into a single directory and compile using: `make`
### 3. Run the Simulation
Run the simulation as follows:
  - **`./nbody`** or **`./nbody -sim1`** to simulate the solar system (by default)
  - **`./nbody -sim3`** to simulate a 3 body problem
  - **`./nbody -sim6`** to simulate a 6 body problem
  - **`./nbody -rand N`** to simulate a random system with N bodies
  - **`./nbody -comp`** to compare the sequential and the concurrent approaches' results
  - **`./nbody -eff N`** to compare the efficiency of the sequential and the concurrent approaches with N random bodies
  - **`./nbody -help`** to see all call options
