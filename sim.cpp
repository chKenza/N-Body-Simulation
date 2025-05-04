#include <gtkmm.h>
#include "nbody.hpp"
#include <thread>
#include <iostream>
#include <vector>
#include <atomic>
#include <chrono>

class SimulationArea: public Gtk::DrawingArea {
    public:
        std::vector<Body> draw_data;

    protected:
        bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr) override {
            // Clear background
            Gtk::Allocation allocation = get_allocation();
            int width = allocation.get_width();
            int height = allocation.get_height();
            cr->set_source_rgb(0, 0, 0);
            cr->paint();

            // Draw each body from draw_data
            cr->set_source_rgb(1, 1, 1);
            for (const auto& body : draw_data) {
                double scaled_x = width / 2 + body.x / 1e9;
                double scaled_y = height / 2 + body.y / 1e9;
                cr->arc(scaled_x, scaled_y, 2.0, 0, 2 * G_PI);
                cr->fill();
            }

            return true;
        }
};

class MainWindow: public Gtk::Window {
    NBodySimulation sim;
    SimulationArea area;
    Glib::Dispatcher dispatcher;
    std::thread sim_thread;
    std::atomic<bool> running;

    public:
        MainWindow() : sim(G, dt), running(true) {
            set_title("N-Body Simulation");
            set_default_size(800, 600);
            add(area);
        
            // Add bodies
            sim.addBody(1.989e30, 0.0, 0.0, 0.0, 0.0);             // Sun
            sim.addBody(3.285e23, 5.79e10, 0.0, 0.0, 47400.0);     // Mercury
            sim.addBody(4.867e24, 1.082e11, 0.0, 0.0, 35000.0);    // Venus
            sim.addBody(5.972e24, 1.496e11, 0.0, 0.0, 29800.0);    // Earth
            sim.addBody(6.39e23, 2.279e11, 0.0, 0.0, 24100.0);     // Mars
            sim.addBody(1.898e27, 7.785e11, 0.0, 0.0, 13070.0);    // Jupiter
            sim.addBody(5.683e26, 1.433e12, 0.0, 0.0, 9690.0);     // Saturn
            sim.addBody(8.681e25, 2.877e12, 0.0, 0.0, 6810.0);     // Uranus
            sim.addBody(1.024e26, 4.503e12, 0.0, 0.0, 5430.0);     // Neptune
                    
            dispatcher.connect(sigc::mem_fun(*this, &MainWindow::on_simulation_step));
            show_all_children();
        
            sim_thread = std::thread([this]() {
                while (running) {
                    sim.stepParallel(4);
                    area.draw_data = sim.getBodies();  // copy for UI
                    dispatcher.emit();                 // trigger redraw
                    std::this_thread::sleep_for(std::chrono::milliseconds(30));
                }
            });
        }
    
        void on_simulation_step() {
            area.queue_draw();  // ask GTK to redraw with latest copy
        }
    
        ~MainWindow() {
            running = false;
            if (sim_thread.joinable()) sim_thread.join();
        }
};

int main(int argc, char *argv[]) {

    // If no argument is provided, default to -sim
    std::string option = (argc < 2) ? "-sim" : argv[1];

    if (option == "-sim") {
        auto app = Gtk::Application::create(argc, argv);
        MainWindow window;
        return app->run(window);

    } else if (option == "-comp") {
        // In case of -comp, run the N-body simulation
        const double G = 6.67430e-11;  // gravitational constant (m^3 kg^-1 s^-2)
        const double dt = 10000;  // time step (s)
        // const double epsilon = 1e-6;

        NBodySimulation sim_seq(G, dt);
        NBodySimulation sim_par(G, dt);

        // Add bodies (same as before)
        sim_seq.addBody(1.989e30, 0.0, 0.0, 0.0, 0.0);  // Sun-like body
        sim_seq.addBody(5.972e24, 1.49e11, 0.0, 0.0, 29800.0);  // Earth-like body
        sim_seq.addBody(6.39e23, 2.28e11, 0.0, 0.0, 24100.0);  // Mars-like body

        sim_par.addBody(1.989e30, 0.0, 0.0, 0.0, 0.0);  // Sun-like body
        sim_par.addBody(5.972e24, 1.49e11, 0.0, 0.0, 29800.0);  // Earth-like body
        sim_par.addBody(6.39e23, 2.28e11, 0.0, 0.0, 24100.0);  // Mars-like body

        const size_t num_threads = 4;

        // Simulate
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

                bool same = compareBodies(bodies_seq, bodies_par);
                std::cout << (same ? "Bodies are the same" : "Bodies are different") << std::endl;
                std::cout << ("----------------------------------------") << std::endl;
            }
        }

        return 0;
    } else {
        std::cerr << "Invalid option: " << option << ". Use -comp or nothing." << std::endl;
        return 1;
    }
}
