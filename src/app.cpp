#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "rod_mechanics/external_forces/dampingForce.h"
#include "rod_mechanics/external_forces/gravityForce.h"
#include "rod_mechanics/external_forces/floorContactForce.h"
#include "rod_mechanics/external_forces/uniformConstantForce.h"
#include "rod_mechanics/external_forces/contactForce.h"

#include "globalDefinitions.h"
#include "world.h"
#include "logging/worldLogger.h"
#include "simulation_environments/headlessDERSimulationEnvironment.h"
#include "simulation_environments/openglDERSimulationEnvironment.h"
#ifdef WITH_MAGNUM
#include "simulation_environments/magnumDERSimulationEnvironment.h"
#endif


namespace py = pybind11;


// Hack: main creates the output file for logging
ofstream logging_output_file;

double openglDERSimulationEnvironment::render_scale = 1.0;
bool openglDERSimulationEnvironment::show_mat_frames = false;

// Forward cmdline args from python
int argc;
std::vector<char*> argv;


// Wrapper for running the simulation from Python
class simulationManager {
public:
    std::shared_ptr<softRobots> soft_robots;
    std::shared_ptr<forceContainer> forces;
    std::shared_ptr<world> my_world;
    simParams sim_params;
    renderParams render_params;
    std::shared_ptr<worldLogger> logger;
    std::unique_ptr<derSimulationEnvironment> env;

    simulationManager() {
        soft_robots = std::make_shared<softRobots>();
        forces = std::make_shared<forceContainer>();
        logger = nullptr;
    }

    void initialize(int argc, char* argv[]) {
        soft_robots->setup();
        my_world = std::make_shared<world>(soft_robots, forces, sim_params);

        switch (render_params.renderer) {
            case HEADLESS:
                env = std::make_unique<headlessDERSimulationEnvironment>(my_world, render_params, logger);
                break;
            case OPENGL:
                env = std::make_unique<openglDERSimulationEnvironment>(my_world, render_params, logger, argc, argv);
                break;
#ifdef WITH_MAGNUM
            case MAGNUM:
                env = std::make_unique<Magnum::magnumDERSimulationEnvironment>(my_world, render_params, logger, argc, argv);
                break;
#endif
            default:
                throw std::runtime_error("Unknown renderer type provided.");
        }
    }
    bool simulationCompleted() {
        return !my_world->simulationRunning();
    }

    void stepSimulation() {
        if (env) {
            env->stepSimulation();
        }
    }

    void runSimulation() {
        if (env) {
            env->runSimulation();
        }
    }
};

class PythonKappaBarController : public baseController
{
public:
    typedef std::function<py::list(double, std::vector<double> const &, std::vector<double> const &)> callback_t;
    PythonKappaBarController(const shared_ptr<softRobots>& softRobots, callback_t callback, int limb)
    : baseController(softRobots->limbs) , callback_(callback), limb_idx(limb) {}
    ~PythonKappaBarController() {}

    void updateTimeStep(double dt) override {
        baseController::updateTimeStep(dt);
        std::vector<double> x(num_actuators),dx(num_actuators);
        // TODO: get state, 
        py::list values = callback_(current_time,x,dx);
        std::vector<double> values_d(values.size());
        for (std::size_t idx=0;idx<values.size();idx++) {
            values_d.at(idx) = values[idx].attr("__float__")().cast<float>();
        }
        auto limb = limbs[limb_idx];
        for (int i = 1; i < limb->ne; i++) {
            for(int n = 0; n<2;n++) {
                limb->kappa_bar(i, n) = values_d.at(n);
            }
        }
    }
protected:
    callback_t callback_;
    int limb_idx;
};

PYBIND11_MODULE(py_dismech, m) {
    py::class_<baseController, std::shared_ptr<baseController>>(m, "baseController");

    py::class_<PythonKappaBarController, baseController, std::shared_ptr<PythonKappaBarController>>(m, "PythonKappaBarController")
        .def(py::init<const shared_ptr<softRobots>&, PythonKappaBarController::callback_t, int>());
    // Wrapper class for accessing dismech data and logic via pybind
    py::class_<simulationManager>(m, "SimulationManager")
        .def(py::init<>())
        .def("initialize", [](simulationManager& self, std::vector<std::string> args) {
            // Convert std::vector<std::string> to argc and argv
            argc = args.size();
            for (int i = 0; i < argc; ++i) {
                argv.push_back(const_cast<char*>(args[i].c_str()));
            }
            argv.push_back(nullptr);  // Null-terminate the argv array
            // Call the original function
            self.initialize(argc, argv.data());
        })
        .def("simulation_completed", &simulationManager::simulationCompleted)
        .def("step_simulation", &simulationManager::stepSimulation)
        .def("run_simulation", &simulationManager::runSimulation)
        .def_readwrite("soft_robots", &simulationManager::soft_robots)
        .def_readwrite("forces", &simulationManager::forces)
        .def_readwrite("sim_params", &simulationManager::sim_params)
        .def_readwrite("render_params", &simulationManager::render_params)
        .def_readwrite("logger", &simulationManager::logger);

    py::class_<softRobots, std::shared_ptr<softRobots>>(m, "SoftRobots")
        .def(py::init<>())
        .def("addLimb",
            py::overload_cast<const Eigen::Vector3d&, const Eigen::Vector3d&,
                              int, double, double, double, double, double>(&softRobots::addLimb),
            py::arg("start"), py::arg("end"), py::arg("num_nodes"),
            py::arg("rho"), py::arg("rod_radius"), py::arg("youngs_modulus"),
            py::arg("poisson_ratio"), py::arg("mu") = 0.0)
        .def("addLimb",
            py::overload_cast<const std::vector<Eigen::Vector3d>&, double, double,
                              double, double, double>(&softRobots::addLimb),
            py::arg("nodes"), py::arg("rho"), py::arg("rod_radius"),
            py::arg("youngs_modulus"), py::arg("poisson_ratio"), py::arg("mu") = 0.0)
        .def("createJoint", &softRobots::createJoint, py::arg("limb_idx"), py::arg("node_idx"))
        .def("addToJoint", &softRobots::addToJoint, py::arg("joint_idx"), py::arg("limb_idx"), py::arg("node_idx"))
        .def("lockEdge", &softRobots::lockEdge, py::arg("limb_idx"), py::arg("edge_idx"))
        .def("applyInitialVelocities", &softRobots::applyInitialVelocities, py::arg("limb_idx"), py::arg("velocities"))
        .def("setup", &softRobots::setup)
        .def("addController", &softRobots::addController, py::arg("controller"))
        .def_readwrite("limbs", &softRobots::limbs)
        .def_readwrite("joints", &softRobots::joints)
        .def_readwrite("controllers", &softRobots::controllers);

    // ====================================== Enum Definitions ============================================
    py::enum_<integratorMethod>(m, "IntegratorMethod")
        .value("FORWARD_EULER", FORWARD_EULER)
        .value("VERLET_POSITION", VERLET_POSITION)
        .value("BACKWARD_EULER", BACKWARD_EULER)
        .value("IMPLICIT_MIDPOINT", IMPLICIT_MIDPOINT)
        .export_values();

    py::enum_<renderEngine>(m, "RenderEngine")
        .value("HEADLESS", HEADLESS)
        .value("OPENGL", OPENGL)
        .value("MAGNUM", MAGNUM)
        .export_values();

    // ===================================== Struct Definitions ============================================
    py::class_<simParams::maxIterations>(m, "MaxIterations")
        .def(py::init<>())
        .def_readwrite("num_iters", &simParams::maxIterations::num_iters)
        .def_readwrite("terminate_at_max", &simParams::maxIterations::terminate_at_max);

    py::class_<simParams>(m, "SimParams")
        .def(py::init<>())
        .def_readwrite("sim_time", &simParams::sim_time)
        .def_readwrite("dt", &simParams::dt)
        .def_readwrite("integrator", &simParams::integrator)
        .def_readwrite("dtol", &simParams::dtol)
        .def_readwrite("ftol", &simParams::ftol)
        .def_readwrite("max_iter", &simParams::max_iter)
        .def_readwrite("line_search", &simParams::line_search)
        .def_readwrite("adaptive_time_stepping", &simParams::adaptive_time_stepping)
        .def_readwrite("enable_2d_sim", &simParams::enable_2d_sim);

    py::class_<renderParams>(m, "RenderParams")
        .def(py::init<>())
        .def_readwrite("renderer", &renderParams::renderer)
        .def_readwrite("render_scale", &renderParams::render_scale)
        .def_readwrite("cmd_line_per", &renderParams::cmd_line_per)
        .def_readwrite("render_per", &renderParams::render_per)
        .def_readwrite("render_record_path", &renderParams::render_record_path)
        .def_readwrite("show_mat_frames", &renderParams::show_mat_frames)
        .def_readwrite("debug_verbosity", &renderParams::debug_verbosity);

    // ====================================== Force Definitions ============================================
    py::class_<forceContainer, std::shared_ptr<forceContainer>>(m, "ForceContainer")
        .def(py::init<>())
        .def(py::init<const std::vector<std::shared_ptr<baseForce>>&>(), py::arg("m_forces"))
        .def("addForce", &forceContainer::addForce, py::arg("force"));

    py::class_<baseForce, std::shared_ptr<baseForce>>(m, "BaseForce");

    py::class_<gravityForce, std::shared_ptr<gravityForce>, baseForce>(m, "GravityForce")
        .def(py::init<const std::shared_ptr<softRobots>&, const Eigen::Vector3d &>(),
            py::arg("soft_robots"), py::arg("g_vector"));

    py::class_<floorContactForce, std::shared_ptr<floorContactForce>, baseForce>(m, "FloorContactForce")
        .def(py::init<const std::shared_ptr<softRobots>&, double, double, double, double>(),
            py::arg("soft_robots"), py::arg("floor_delta"), py::arg("floor_slipTol"),
            py::arg("floor_z"), py::arg("floor_mu") = 0.0);
}
