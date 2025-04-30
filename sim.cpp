#include <gtkmm.h>
#include "nbody.hpp"
#include <thread>
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
            sim.addBody(1.989e30, 0.0, 0.0, 0.0, 0.0); // sun
            sim.addBody(5.972e24, 1.49e11, 0.0, 0.0, 29800.0); // earth
            sim.addBody(6.39e23, 2.28e11, 0.0, 0.0, 24100.0); // mars
        
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
    // Initialize GTK application
    auto app = Gtk::Application::create(argc, argv);

    // Create main window
    MainWindow window;

    // Run the application
    return app->run(window);
}
