#include <gtkmm.h>
#include "nbody.hpp"
#include <thread>
#include <iostream>
#include <vector>
#include <atomic>
#include <chrono>
#include <string>
#include <cstdlib>

double random_double(double min, double max) {
    return min + (max - min) * (rand() / (RAND_MAX + 1.0));
}

class SimulationArea: public Gtk::DrawingArea {
    public:
        std::vector<Body> draw_data;
        double zoom_factor = 1.0;
        double offset_x = 0.0;
        double offset_y = 0.0;
        bool dragging = false;
        double drag_start_x = 0.0;
        double drag_start_y = 0.0;

        SimulationArea() {
            add_events(Gdk::SCROLL_MASK | Gdk::KEY_PRESS_MASK | 
                      Gdk::BUTTON_PRESS_MASK | Gdk::BUTTON_RELEASE_MASK | 
                      Gdk::BUTTON_MOTION_MASK | Gdk::POINTER_MOTION_MASK);
        }

    protected:
        bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr) override {
            Gtk::Allocation allocation = get_allocation();
            int width = allocation.get_width();
            int height = allocation.get_height();
            cr->set_source_rgb(0, 0, 0);
            cr->paint();

            for (const auto& body : draw_data) {
                cr->set_source_rgb(body.r, body.g, body.b);
                double scaled_x = (width / 2 + body.x / 1e9 + offset_x) * zoom_factor;
                double scaled_y = (height / 2 + body.y / 1e9 + offset_y) * zoom_factor;
                cr->arc(scaled_x, scaled_y, (2.0 + 4.0 * (std::log10(body.mass) - 20.0) / 5.0) * zoom_factor, 0, 2 * G_PI); // Scaled size based on mass
                cr->fill();
            }

            return true;
        }

        bool on_scroll_event(GdkEventScroll* scroll_event) override {
            if (scroll_event->direction == GDK_SCROLL_UP) {
                zoom_factor *= 1.2; // Zoom in
            } else if (scroll_event->direction == GDK_SCROLL_DOWN) {
                zoom_factor /= 1.2; // Zoom out
            }
            queue_draw();
            return true;
        }

        bool on_button_press_event(GdkEventButton* button_event) override {
            if (button_event->button == 1) { // Left mouse button
                dragging = true;
                drag_start_x = button_event->x;
                drag_start_y = button_event->y;
                return true;
            }
            return false;
        }

        bool on_button_release_event(GdkEventButton* button_event) override {
            if (button_event->button == 1) { // Left mouse button
                dragging = false;
                return true;
            }
            return false;
        }

        bool on_motion_notify_event(GdkEventMotion* motion_event) override {
            if (dragging) {
                double dx = motion_event->x - drag_start_x;
                double dy = motion_event->y - drag_start_y;
                
                // Update offset based on the drag distance
                offset_x += dx / zoom_factor;
                offset_y += dy / zoom_factor;
                
                // Reset drag start position
                drag_start_x = motion_event->x;
                drag_start_y = motion_event->y;
                
                queue_draw();
                return true;
            }
            return false;
        }
};

class MainWindow: public Gtk::Window {
    NBodySimulation sim;
    SimulationArea area;
    Glib::Dispatcher dispatcher;
    std::thread sim_thread;
    std::atomic<bool> running;
    std::atomic<bool> paused;

    Gtk::Box main_box;
    Gtk::Box control_box;
    Gtk::Label time_step_label;
    Gtk::Button slower_button;
    Gtk::Button faster_button;
    Gtk::Button pause_button;
    Gtk::Button zoom_in_button;
    Gtk::Button zoom_out_button;
    Gtk::Button reset_sim_button;
    Gtk::Label zoom_label;
    double dt_current;
    int sim_type;
    int random_bodies;

    public:
        MainWindow(int sim_type = 1, int random_bodies = 0) : 
            sim(G, dt), 
            running(true), 
            paused(false),
            main_box(Gtk::ORIENTATION_VERTICAL),
            control_box(Gtk::ORIENTATION_HORIZONTAL, 5),
            slower_button("Slower"),
            faster_button("Faster"),
            pause_button("Pause"),
            zoom_in_button("Zoom In"),
            zoom_out_button("Zoom Out"),
            reset_sim_button("Reset"),
            dt_current(dt),
            sim_type(sim_type),
            random_bodies(random_bodies)
        {
            set_title("N-Body Simulation");
            set_default_size(800, 600);
    
            add(main_box);
            main_box.pack_start(control_box, Gtk::PACK_SHRINK);
            main_box.pack_start(area, Gtk::PACK_EXPAND_WIDGET);
    
            control_box.pack_start(slower_button, Gtk::PACK_SHRINK);
            control_box.pack_start(time_step_label, Gtk::PACK_SHRINK);
            control_box.pack_start(faster_button, Gtk::PACK_SHRINK);
            control_box.pack_start(pause_button, Gtk::PACK_SHRINK);
            control_box.pack_start(zoom_out_button, Gtk::PACK_SHRINK);
            control_box.pack_start(zoom_label, Gtk::PACK_SHRINK);
            control_box.pack_start(zoom_in_button, Gtk::PACK_SHRINK);
            control_box.pack_start(reset_sim_button, Gtk::PACK_SHRINK);
        
            slower_button.signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::on_slower_clicked));
            faster_button.signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::on_faster_clicked));
            pause_button.signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::on_pause_clicked));
            zoom_in_button.signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::on_zoom_in_clicked));
            zoom_out_button.signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::on_zoom_out_clicked));
            reset_sim_button.signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::on_reset_sim_clicked));
    
            add_events(Gdk::KEY_PRESS_MASK);
            signal_key_press_event().connect(sigc::mem_fun(*this, &MainWindow::on_key_press_event), false);

            setup_sim(sim_type, random_bodies);
            
            dispatcher.connect(sigc::mem_fun(*this, &MainWindow::on_simulation_step));
            show_all_children();

            sim_thread = std::thread([this]() {
                while (running) {
                    if (!paused) {
                        sim.setTimeStep(dt_current);
                        sim.stepParallel(4);
                        area.draw_data = sim.getBodies(); 
                        dispatcher.emit();
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(30));
                }
            });        
        }

        void setup_sim(int sim_type, int random_bodies){
            if (sim_type == 1) {
                setupSolarSystem();
            } else if (sim_type == 3) {
                setupThreeBody();
            } else if (sim_type == 6) {
                setupHexagonBodies();
            } else if (random_bodies > 0) {
                setupRandomBodies(random_bodies);
            }
        }
    
        void setupSolarSystem() {
            sim.addBody(1.989e30, 0.0, 0.0, 0.0, 0.0, 1.0, 0.84, 0.0);              // Sun
            sim.addBody(3.285e23, 5.79e10, 0.0, 0.0, 47400.0, 0.55, 0.53, 0.52);    // Mercury
            sim.addBody(4.867e24, 1.082e11, 0.0, 0.0, 35000.0, 0.90, 0.75, 0.35);   // Venus
            sim.addBody(5.972e24, 1.496e11, 0.0, 0.0, 29800.0, 0.0, 0.5, 1.0);      // Earth
            sim.addBody(6.39e23, 2.279e11, 0.0, 0.0, 24100.0, 0.8, 0.3, 0.1);       // Mars
            sim.addBody(1.898e27, 7.785e11/2, 0.0, 0.0, 13070.0, 0.85, 0.52, 0.35); // Jupiter
            sim.addBody(5.683e26, 1.433e12/2, 0.0, 0.0, 9690.0, 0.93, 0.82, 0.62);  // Saturn
            sim.addBody(8.681e25, 2.877e12/2, 0.0, 0.0, 6810.0, 0.65, 0.85, 0.88);  // Uranus
            sim.addBody(1.024e26, 4.503e12/2, 0.0, 0.0, 5430.0, 0.25, 0.41, 0.88);  // Neptune
        }
                
        void setupThreeBody() {
            // Classic three-body problem with three stars of similar mass
            // in an equilateral triangle with small random velocity perturbations
                    
            double m = 1.5e27;
            double d = 1.0e11;
            double max_init_velocity = 4.0e3;
            
            double colors[3][3] = {
                {1.0, 0.0, 0.0},
                {0.0, 1.0, 0.0},
                {0.0, 0.0, 1.0}
            };
                    
            for (int i = 0; i < 3; i++) {
                double angle = i * (2.0 * G_PI / 3.0);
                double x = d * cos(angle);
                double y = d * sin(angle);
                
                double dev = random_double(-1.0e3, 1.0e3);
                
                sim.addBody(m + dev, x, y,
                            random_double(-max_init_velocity, max_init_velocity),
                            random_double(-max_init_velocity, max_init_velocity),
                            colors[i][0], colors[i][1], colors[i][2]);
            }
        }        

        void setupHexagonBodies() {
            // Setup six bodies positioned at the vertices of a regular hexagon
            // with small random velocity perturbations
            
            double m = 1.5e27;
            double d = 1.0e11;
            double max_init_velocity = 4.0e3;
            
            double colors[6][3] = {
                {1.0, 0.0, 0.0},
                {1.0, 0.5, 0.0},
                {1.0, 1.0, 0.0},
                {0.0, 1.0, 0.0},
                {0.0, 0.0, 1.0},
                {0.5, 0.0, 0.5} 
            };
            
            // Create six bodies positioned at the vertices of a regular hexagon
            for (int i = 0; i < 6; i++) {
                double angle = i * (2.0 * G_PI / 6.0);
                double x = d * cos(angle);
                double y = d * sin(angle);
                
                double dev = random_double(-1.0e3, 1.0e3);
                
                sim.addBody(m + dev, x, y,
                            random_double(-max_init_velocity, max_init_velocity),
                            random_double(-max_init_velocity, max_init_velocity),
                            colors[i][0], colors[i][1], colors[i][2]);
            }
        }


        void setupRandomBodies(int num_bodies) {
            sim.addBody(1.989e30, 0.0, 0.0, 0.0, 0.0, 1.0, 0.84, 0.0); // Sun
            for (int i = 0; i < num_bodies; ++i) {
                double mass = random_double(1e20, 1e28);
                
                double max_d = 1e12;
                double x = random_double(-max_d, max_d);
                double y = random_double(-max_d, max_d);
                
                double max_velocity = 5e4;
                double vx = random_double(-max_velocity, max_velocity);
                double vy = random_double(-max_velocity, max_velocity);
                

                // Styling to see the big ones better

                double brightness = 0.5 + 0.5 * (std::log10(mass) - 20.0) / 11.0;
                brightness = std::min(1.0, std::max(0.0, brightness));
                
                double r = random_double(0.0, 1.0) * brightness + (1.0 - brightness) * 0.3;
                double g = random_double(0.0, 1.0) * brightness + (1.0 - brightness) * 0.3;
                double b = random_double(0.0, 1.0) * brightness + (1.0 - brightness) * 0.3;
                
                if (mass > 1e29) {
                    r = 1.0;
                    g = random_double(0.7, 1.0);
                    b = random_double(0.0, 0.5);
                }
        
                sim.addBody(mass, x, y, vx, vy, r, g, b);
            }
        }
        
        void on_slower_clicked() {
            dt_current /= 10;
            update_time_step_label();
        }
        
        void on_faster_clicked() {
            dt_current *= 10;
            update_time_step_label();
        }
        
        void on_pause_clicked() {
            paused = !paused;
            pause_button.set_label(paused ? "Resume" : "Pause");
        }
        
        void on_zoom_in_clicked() {
            area.zoom_factor *= 1.2;
            update_zoom_label();
            area.queue_draw();
        }
        
        void on_zoom_out_clicked() {
            area.zoom_factor /= 1.2;
            update_zoom_label();
            area.queue_draw();
        }
        
        void on_reset_sim_clicked() {
            sim.clear();
            setup_sim(sim_type, random_bodies);
        }
        
        void update_time_step_label() {
            time_step_label.set_text("Time Step: " + std::to_string(dt_current));
        }
        
        void update_zoom_label() {
            zoom_label.set_text("Zoom: " + std::to_string(area.zoom_factor).substr(0, 4) + "x");
        }
        
        bool on_key_press_event(GdkEventKey* event) {
            switch(event->keyval) {
                case GDK_KEY_plus:
                case GDK_KEY_equal:
                    on_zoom_in_clicked();
                    return true;
                case GDK_KEY_minus:
                    on_zoom_out_clicked();
                    return true;
                case GDK_KEY_space:
                    on_pause_clicked();
                    return true;
                case GDK_KEY_Up:
                    on_faster_clicked();
                    return true;
                case GDK_KEY_Down:
                    on_slower_clicked();
                    return true;
                case GDK_KEY_r:
                    on_reset_sim_clicked();
                    return true;
            }
            return false;
        }
        
        void on_simulation_step() {
            update_time_step_label();
            update_zoom_label();
            area.queue_draw();
        }

        ~MainWindow() {
            running = false;
            if (sim_thread.joinable()) sim_thread.join();
        }
};

//Test the accuracy of the parallel approach
void runComparison() {
    const double G = 6.67430e-11;
    const double dt = 10000;
    const size_t num_threads = 32;

    NBodySimulation sim_seq(G, dt);
    NBodySimulation sim_par(G, dt);

    // Add random bodies
    for (int i = 0; i < 100; ++i) {
        double mass = random_double(1e22, 1e27);
        double x = random_double(1e10, 1e12);
        double vy = random_double(1e3, 1e4);
        double r = random_double(0.0, 1.0);
        double g = random_double(0.0, 1.0);
        double b = random_double(0.0, 1.0);

        sim_seq.addBody(mass, x, 0.0, 0.0, vy, r, g, b);
        sim_par.addBody(mass, x, 0.0, 0.0, vy, r, g, b);
    }

    for (int step = 0; step <= 1000; ++step) {
        sim_seq.stepSequential();
        sim_par.stepParallel(num_threads);

        if (step % 100 == 0) {
            const auto& bodies_seq = sim_seq.getBodies();
            const auto& bodies_par = sim_par.getBodies();
            double error = 0.0;
            PositionRelativeError(bodies_seq, bodies_par, error);

            std::cout << "Step " << step << ":\n";
            std::cout << "Position Relative Error: " << 100*error << "%" << "\n";
            std::cout << "----------------------------------------\n";
        }
    }

}

// Test for efficiency
void runEfficiencyTest(int num_bodies) {
    const double G = 6.67430e-11;
    const double dt = 10000;
    const int steps = 1000;
    int num_threads = 4;

    for (int k = 0; k < 8; ++k){ // To see which number of threads is better depending on N bodies ||| TESTING ONLY |||

    NBodySimulation sim_seq(G, dt);
    NBodySimulation sim_par(G, dt);

    for (int i = 0; i < num_bodies; ++i) {
        double mass = random_double(1e20, 1e28);
        double x = random_double(-1e12, 1e12);
        double y = random_double(-1e12, 1e12);
        double vx = random_double(-5e4, 5e4);
        double vy = random_double(-5e4, 5e4);

        sim_seq.addBody(mass, x, y, vx, vy);
        sim_par.addBody(mass, x, y, vx, vy);
    }

    // Time sequential way
    auto start_seq = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < steps; ++i) {
        sim_seq.stepSequential();
    }
    auto end_seq = std::chrono::high_resolution_clock::now();
    double time_seq = std::chrono::duration<double>(end_seq - start_seq).count();

    // Time parallel way
    auto start_par = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < steps; ++i) {
        sim_par.stepParallel(num_threads);
    }
    auto end_par = std::chrono::high_resolution_clock::now();
    double time_par = std::chrono::duration<double>(end_par - start_par).count();

    std::cout << "=== Efficiency Report ===" << std::endl;
    std::cout << "Sequential time: " << time_seq << " s" << std::endl;
    std::cout << "Parallel time (" << num_threads << " threads): " << time_par << " s" << std::endl;
    std::cout << "Speedup: " << time_seq / time_par << "x" << std::endl;
    num_threads *= 2;
}
}


int main(int argc, char *argv[]) {
    // Process args before GTK to avoid errors
    int sim_type = 1;  // Default to solar system
    int random_bodies = 0;
    bool run_comp = false;
    bool show_help = false;
    bool run_efficiency = false;
    int efficiency_bodies = 0;
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "-sim1") {
            sim_type = 1;
            for(int j = i; j < argc-1; j++) {
                argv[j] = argv[j+1];
            }
            argc--;
            i--;
        }
        else if (arg == "-sim3") {
            sim_type = 3;
            for(int j = i; j < argc-1; j++) {
                argv[j] = argv[j+1];
            }
            argc--;
            i--;
        }
        else if (arg == "-sim6") {
            sim_type = 6;
            for(int j = i; j < argc-1; j++) {
                argv[j] = argv[j+1];
            }
            argc--;
            i--;
        }

        else if (arg == "-comp") {
            run_comp = true;
            for(int j = i; j < argc-1; j++) {
                argv[j] = argv[j+1];
            }
            argc--;
            i--;
        }
        else if (arg == "-eff" && i + 1 < argc) {
            try {
                efficiency_bodies = std::stoi(argv[i + 1]);
                if (efficiency_bodies <= 0) {
                    std::cerr << "Invalid number of bodies." << std::endl;
                    show_help = true;
                    break;
                }
                run_efficiency = true;
                for (int j = i; j < argc - 2; j++) {
                    argv[j] = argv[j + 2];
                }
                argc -= 2;
                i--;
            } catch (const std::exception& e) {
                std::cerr << "Error parsing efficiency test argument: " << e.what() << std::endl;
                show_help = true;
                break;
            }
        }
        
        else if (arg == "-rand" && i+1 < argc) {
            try {
                random_bodies = std::stoi(argv[i+1]);
                if (random_bodies <= 0) {
                    std::cerr << "Cannot have negative number of bodies." << std::endl;
                    show_help = true;
                    break;
                }
                
                for(int j = i; j < argc-2; j++) {
                    argv[j] = argv[j+2];
                }
                argc -= 2;
                i--;
            }
            catch (const std::exception& e) {
                std::cerr << "Error parsing number of bodies: " << e.what() << std::endl;
                show_help = true;
                break;
            }
        }
        else if (arg == "-help") {
            show_help = true;
        }
    }
    
    if (show_help) {
        std::cout << "Usage: " << argv[0] << " [OPTION]" << std::endl;
        std::cout << "Options:" << std::endl;
        std::cout << "  |  -sim1       Solar System Simulation (DEFAULT)" << std::endl;
        std::cout << "  |  -sim3       3 Body Simulation" << std::endl;
        std::cout << "  |  -sim6       [BROKEN] 6 Body Simulation" << std::endl;
        std::cout << "  |  -rand N     Simulation with N random bodies" << std::endl;
        std::cout << "  |  -comp       Accuracy test for the parallel approach using position relative error on 100 random bodies" << std::endl;
        std::cout << "  |  -eff N      Efficiency test with N random bodies" << std::endl;
        std::cout << "  |  -help       Show this Help" << std::endl;
        return 0;
    }
    
    if (run_comp) {
        runComparison();
        return 0;
    }
    if (run_efficiency) {
        runEfficiencyTest(efficiency_bodies);
        return 0;
    }
    else if (random_bodies > 0) {
        auto app = Gtk::Application::create(argc, argv);
        MainWindow window(0, random_bodies); // Random bodies Sim
        return app->run(window);
    }
    else {
        auto app = Gtk::Application::create(argc, argv);
        MainWindow window(sim_type);         // General Sims
        return app->run(window);
    }
}
