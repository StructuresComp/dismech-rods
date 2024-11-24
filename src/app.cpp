#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "rod_mechanics/elastic_joint.h"
#include "rod_mechanics/elastic_rod.h"
#include "rod_mechanics/external_forces/damping_force.h"
#include "rod_mechanics/external_forces/floor_contact_force.h"
#include "rod_mechanics/external_forces/gravity_force.h"
#include "rod_mechanics/force_container.h"
#include "rod_mechanics/soft_robots.h"

#include "global_definitions.h"
#include "logging/base_logger.h"
#include "simulation_environments/headless_sim_env.h"
#include "simulation_environments/opengl_sim_env.h"
#include "world.h"
#ifdef WITH_MAGNUM
#include "simulation_environments/magnum_sim_env.h"
#endif

namespace py = pybind11;

// Hack: main creates the output file for logging
std::ofstream logging_output_file;

double OpenGLSimEnv::render_scale = 1.0;
bool OpenGLSimEnv::show_mat_frames = false;

// Forward cmdline args from python
int argc;
std::vector<char*> argv;

// Wrapper for running the simulation from Python
class SimulationManager
{
  public:
    std::shared_ptr<SoftRobots> soft_robots;
    std::shared_ptr<ForceContainer> forces;
    std::shared_ptr<World> my_world;
    SimParams sim_params;
    RenderParams render_params;
    std::shared_ptr<BaseLogger> logger;
    std::unique_ptr<BaseSimEnv> env;

    SimulationManager() {
        soft_robots = std::make_shared<SoftRobots>();
        forces = std::make_shared<ForceContainer>();
        logger = nullptr;
    }

    void initialize(int argc, char* argv[]) {
        soft_robots->setup();
        my_world = std::make_shared<World>(soft_robots, forces, sim_params);

        switch (render_params.renderer) {
            case HEADLESS:
                env = std::make_unique<HeadlessSimEnv>(my_world, render_params, logger);
                break;
            case OPENGL:
                env = std::make_unique<OpenGLSimEnv>(my_world, render_params, logger, argc, argv);
                break;
#ifdef WITH_MAGNUM
            case MAGNUM:
                env = std::make_unique<Magnum::MagnumSimEnv>(my_world, render_params, logger, argc,
                                                             argv);
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

    // add the step function with the input variables
    void stepSimulation(const py::dict& input_dict) {
        for (const auto& [key_obj, value_obj] : input_dict) {
            std::string key = py::str(key_obj);
            MatX values = value_obj.cast<MatX>();

            if (values.cols() == 1) {
                values.transposeInPlace();
            }
            if (key == "position") {
                if (values.cols() != 5) {
                    throw std::invalid_argument("Position BC values must have 5 columns");
                }
                soft_robots->applyPositionBC(values);
            }
            if (key == "delta_position") {
                if (values.cols() != 5) {
                    throw std::invalid_argument("Delta position BC values must have 5 columns");
                }
                soft_robots->applyDeltaPositionBC(values);
            }
            else if (key == "theta") {
                if (values.cols() != 3) {
                    throw std::invalid_argument("Theta BC values must have 3 columns");
                }
                soft_robots->applyThetaBC(values);
            }
            else if (key == "delta_theta") {
                if (values.cols() != 3) {
                    throw std::invalid_argument("Delta theta BC values must have 3 columns");
                }
                soft_robots->applyDeltaThetaBC(values);
            }
            else if (key == "curvature") {
                if (values.cols() != 4) {
                    throw std::invalid_argument("Curvature BC values must have 4 columns");
                }
                soft_robots->applyCurvatureBC(values);
            }
            else if (key == "delta_curvature") {
                if (values.cols() != 4) {
                    throw std::invalid_argument("Delta curvature BC values must have 4 columns");
                }
                soft_robots->applyDeltaCurvatureBC(values);
            }
            else {
                throw std::invalid_argument("Invalid input key: " + key);
            }
        }
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

PYBIND11_MODULE(py_dismech, m) {
    // Wrapper class for accessing dismech data and logic via pybind
    py::class_<SimulationManager>(m, "SimulationManager")
        .def(py::init<>())
        .def("initialize",
             [](SimulationManager& self, std::vector<std::string> args) {
                 // Convert std::vector<std::string> to argc and argv
                 argc = args.size();
                 for (int i = 0; i < argc; ++i) {
                     argv.push_back(const_cast<char*>(args[i].c_str()));
                 }
                 argv.push_back(nullptr);  // Null-terminate the argv array
                 // Call the original function
                 self.initialize(argc, argv.data());
             })
        .def("simulation_completed", &SimulationManager::simulationCompleted)
        .def("step_simulation", py::overload_cast<>(&SimulationManager::stepSimulation))
        .def("step_simulation",
             py::overload_cast<const py::dict&>(&SimulationManager::stepSimulation))
        .def("run_simulation", &SimulationManager::runSimulation)
        .def_readonly("soft_robots", &SimulationManager::soft_robots)
        .def_readonly("forces", &SimulationManager::forces)
        .def_readonly("sim_params", &SimulationManager::sim_params)
        .def_readonly("render_params", &SimulationManager::render_params)
        .def_readwrite("logger", &SimulationManager::logger);

    py::class_<SoftRobots, std::shared_ptr<SoftRobots>>(m, "SoftRobots")
        .def(py::init<>())
        .def("addLimb",
             py::overload_cast<const Vec3&, const Vec3&, int, double, double, double, double,
                               double>(&SoftRobots::addLimb),
             py::arg("start"), py::arg("end"), py::arg("num_nodes"), py::arg("rho"),
             py::arg("rod_radius"), py::arg("youngs_modulus"), py::arg("poisson_ratio"),
             py::arg("mu") = 0.0)
        .def("addLimb",
             py::overload_cast<const std::vector<Vec3>&, double, double, double, double, double>(
                 &SoftRobots::addLimb),
             py::arg("nodes"), py::arg("rho"), py::arg("rod_radius"), py::arg("youngs_modulus"),
             py::arg("poisson_ratio"), py::arg("mu") = 0.0)
        .def("createJoint", &SoftRobots::createJoint, py::arg("limb_idx"), py::arg("node_idx"))
        .def("addToJoint", &SoftRobots::addToJoint, py::arg("joint_idx"), py::arg("limb_idx"),
             py::arg("node_idx"))
        .def("lockEdge", &SoftRobots::lockEdge, py::arg("limb_idx"), py::arg("edge_idx"))
        .def("applyInitialVelocities", &SoftRobots::applyInitialVelocities, py::arg("limb_idx"),
             py::arg("velocities"))
        .def("setup", &SoftRobots::setup)
        .def_readonly("limbs", &SoftRobots::limbs, py::return_value_policy::reference_internal)
        .def_readonly("joints", &SoftRobots::joints, py::return_value_policy::reference_internal);

    py::class_<ElasticRod, std::shared_ptr<ElasticRod>>(m, "ElasticRod")
        .def("getVertexPos", &ElasticRod::getVertex)
        .def("getVertexVel", &ElasticRod::getVelocity)
        .def("getEdgeTheta", &ElasticRod::getTheta)
        .def("getVertices", &ElasticRod::getVertices)
        .def("getVelocities", &ElasticRod::getVelocities)
        .def("getThetas", &ElasticRod::getThetas)
        .def("freeVertexBoundaryCondition", &ElasticRod::freeVertexBoundaryCondition)
        .def("setVertexBoundaryCondition", &ElasticRod::setVertexBoundaryCondition)
        .def("setThetaBoundaryCondition", &ElasticRod::setThetaBoundaryCondition);

    // =============================== Enum Definitions =========================================
    py::enum_<IntegratorMethod>(m, "IntegratorMethod")
        .value("FORWARD_EULER", FORWARD_EULER)
        .value("VERLET_POSITION", VERLET_POSITION)
        .value("BACKWARD_EULER", BACKWARD_EULER)
        .value("IMPLICIT_MIDPOINT", IMPLICIT_MIDPOINT)
        .export_values();

    py::enum_<RenderEngine>(m, "RenderEngine")
        .value("HEADLESS", HEADLESS)
        .value("OPENGL", OPENGL)
        .value("MAGNUM", MAGNUM)
        .export_values();

    py::enum_<LineSearchType>(m, "LineSearchType")
        .value("NO_LS", NO_LS)
        .value("GOLDSTEIN", GOLDSTEIN)
        .value("WOLFE", WOLFE)
        .export_values();

    // ================================== Struct Definitions =====================================
    py::class_<SimParams::MaxIterations>(m, "MaxIterations")
        .def(py::init<>())
        .def_readwrite("num_iters", &SimParams::MaxIterations::num_iters)
        .def_readwrite("terminate_at_max", &SimParams::MaxIterations::terminate_at_max);

    py::class_<SimParams>(m, "SimParams")
        .def(py::init<>())
        .def_readwrite("sim_time", &SimParams::sim_time)
        .def_readwrite("dt", &SimParams::dt)
        .def_readwrite("integrator", &SimParams::integrator)
        .def_readwrite("dtol", &SimParams::dtol)
        .def_readwrite("ftol", &SimParams::ftol)
        .def_readwrite("max_iter", &SimParams::max_iter)
        .def_readwrite("line_search", &SimParams::line_search)
        .def_readwrite("adaptive_time_stepping", &SimParams::adaptive_time_stepping)
        .def_readwrite("enable_2d_sim", &SimParams::enable_2d_sim);

    py::class_<RenderParams>(m, "RenderParams")
        .def(py::init<>())
        .def_readwrite("renderer", &RenderParams::renderer)
        .def_readwrite("render_scale", &RenderParams::render_scale)
        .def_readwrite("cmd_line_per", &RenderParams::cmd_line_per)
        .def_readwrite("render_per", &RenderParams::render_per)
        .def_readwrite("render_record_path", &RenderParams::render_record_path)
        .def_readwrite("show_mat_frames", &RenderParams::show_mat_frames)
        .def_readwrite("debug_verbosity", &RenderParams::debug_verbosity);

    // ============================== Force Definitions ==========================================
    py::class_<ForceContainer, std::shared_ptr<ForceContainer>>(m, "ForceContainer")
        .def(py::init<>())
        .def(py::init<const std::vector<std::shared_ptr<BaseForce>>&>(), py::arg("m_forces"))
        .def("addForce", &ForceContainer::addForce, py::arg("force"));

    py::class_<BaseForce, std::shared_ptr<BaseForce>>(m, "BaseForce");

    py::class_<DampingForce, std::shared_ptr<DampingForce>, BaseForce>(m, "DampingForce")
        .def(py::init<const std::shared_ptr<SoftRobots>&, double>(), py::arg("soft_robots"),
             py::arg("viscosity"));

    py::class_<GravityForce, std::shared_ptr<GravityForce>, BaseForce>(m, "GravityForce")
        .def(py::init<const std::shared_ptr<SoftRobots>&, const Vec3&>(), py::arg("soft_robots"),
             py::arg("g_vector"));

    py::class_<FloorContactForce, std::shared_ptr<FloorContactForce>, BaseForce>(
        m, "FloorContactForce")
        .def(py::init<const std::shared_ptr<SoftRobots>&, double, double, double, double>(),
             py::arg("soft_robots"), py::arg("floor_delta"), py::arg("floor_slipTol"),
             py::arg("floor_z"), py::arg("floor_mu") = 0.0);
}
