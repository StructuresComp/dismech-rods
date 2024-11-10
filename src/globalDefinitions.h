#ifndef GLOBAL_DEFINITIONS_H
#define GLOBAL_DEFINITIONS_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Sparse>

#include <cmath>
#include <csignal>
#include <cstdlib>
#include <fcntl.h>
#include <fcntl.h> /* For O_* constants */
#include <iostream>
#include <memory>
#include <sys/mman.h>
#include <sys/stat.h> /* For mode constants */
#include <unistd.h>

using namespace std;
using namespace Eigen;

typedef enum {
    FORWARD_EULER,
    VERLET_POSITION,
    BACKWARD_EULER,
    IMPLICIT_MIDPOINT
} integratorMethod;

typedef enum {
    HEADLESS,
    OPENGL,
    MAGNUM
} renderEngine;

typedef enum {
    NO_LS,
    GOLDSTEIN,
    WOLFE
} lineSearchType;

/**
 * @brief Parameters for simulation.
 */
struct simParams
{
    /**
     * @brief Total time for the simulation [s]
     */
    double sim_time = 10;

    /**
     * @brief Time step size [s]
     *
     * General rule of thumb: explicit methods will require smaller time steps
     * while implicit methods can generally handle larger ones. If simulation
     * crashes or cannot converge, lowering the time step is often effective.
     */
    double dt = 1e-3;

    /**
     * @brief The numerical integration scheme.
     *
     * Available options currently are
     *  - FORWARD_EULER:      Explicit first-order method.
     *                        https://en.wikipedia.org/wiki/Euler_method
     *  - VERLET_POSITION:    Explicit second-order symplectic method.
     *                        https://en.wikipedia.org/wiki/Verlet_integration
     *  - BACKWARD_EULER:     Implicit first-order method.
     *                        https://en.wikipedia.org/wiki/Backward_Euler_method
     *  - IMPLICIT_MIDPOINT:  Implicit second-order symplectic method.
     *                        https://en.wikipedia.org/wiki/Midpoint_method
     */
    integratorMethod integrator = BACKWARD_EULER;

    /**
     * @brief Dynamics tolerance [m/s]
     *
     * Only used if an implicit integration scheme is used.
     * Considers Newton's method to be converged if the infinity norm of the DOF
     * update divided by the time step is below the this tolerance.
     */
    double dtol = 1e-2;

    /**
     * @brief Force tolerance
     *
     * Only used if an implicit integration scheme is used.
     * Considers Newton's method to be converged if the norm of the force during
     * a Newton step is below the norm of the force at the beginning of the time
     * step times this tolerance.
     */
    double ftol = 1e-4;

    /**
     * @brief Maximum number of iterations for a time step.
     *
     * Only used if an implicit integration scheme is used.
     */
    struct maxIterations
    {
        // The number of iterations to attempt Newton's method without
        // converging.
        int num_iters = 500;
        // Whether to terminate at the end of num_iters, or continue to the next
        // time step.
        bool terminate_at_max = true;
    };

    maxIterations max_iter;

    /**
     * @brief The algorithm to use for line search.
     *
     * Only used if an implicit integration scheme is used.
     * Enables line search during Newton's method to adaptively set the step
     * size.
     *
     * Available options currently are
     *  - NO_LS:       Don't conduct line search.
     *  - GOLDSTEIN:   Line search with Goldstein condition.
     *                 https://en.wikipedia.org/wiki/Backtracking_line_search
     *  - WOLFE:       Line search with Wolfe condition
     *                 https://en.wikipedia.org/wiki/Wolfe_conditions
     */
    lineSearchType line_search = GOLDSTEIN;

    /**
     * @brief Adaptive time stepping.
     *
     * Only used if an implicit integration scheme is used.
     * If N > 0, turns on adaptive time stepping which halves the time step size
     * if Newton's method fails to converge after N number of iterations. This
     * will repeat every N iterations until the time step converges. Note that
     * this can cause recorded data to be non-uniform in time. Set to 0 to
     * disable.
     */
    int adaptive_time_stepping = 0;  // Adaptive time stepping*

    /**
     * @brief Switch to 2D simulation.
     *
     * Locks y and theta DOFs, effectively reducing the number of DOFs by
     * approximately half. In other words, the simulation will only operate in
     * the xz plane.
     */
    bool enable_2d_sim = false;
};

/**
 * @brief Parameters for rendering.
 */
struct renderParams
{
    /**
     * @brief Renderer type.
     *
     * Available options currently are
     *  - HEADLESS:  No rendering. Fastest with no overhead. Recommended for
     * generating data.
     *  - OPENGL:    Barebones rendering of rod centerlines with small overhead.
     *               Recommended for general development / debugging.
     *  - MAGNUM:    Renders rod and environment with full 3D meshes and shading. Also supports
     *               camera pose manipulation via mouse. High computational overhead.
     *               Recommended only for creating videos.
     */
    renderEngine renderer = OPENGL;

    /**
     * @brief Scale factor applied to Cartesian DOFs before rendering.
     *
     * Applies a scaling to all Cartesian DOFs.
     * Useful for rendering certain systems that are overly small or large.
     */
    double render_scale = 1.0;

    /**
     * @brief Command line sim info output period.
     *
     * Specifies how often simulation information is printed to the command
     * line. For example, setting this to 5 means that information is printed
     * every 5 simulation time steps. A value of 1 prints information at every
     * time step.
     * */
    int cmd_line_per = 1;

    /**
     * @brief Rendering period
     *
     * TODO: implement for OpenGL.
     *
     * Currently only supported for when renderer is set to MAGNUM.
     * Specifies how often a new frame is rendered.
     * For example, setting this to 5 means that a new frame is printed every 5
     * simulation time steps. This is especially useful when explicit
     * integration is used, where the cost of rendering for very fine time steps
     * can start to add up.
     */
    int render_per = 1;

    /**
     * @brief Rendering frames recording path.
     *
     * Currently only supported for when renderer is set to MAGNUM.
     * If a non-empty string, frames will be recorded to this path as png files.
     */
    string render_record_path;

    /**
     * @brief Show material frames (only for OpenGL)
     *
     * Currently only supported for when renderer is set to OPENGL.
     * If True, the material frames for each discrete edge will be rendered to
     * better showcase bending and twisting deformations.
     */
    bool show_mat_frames = false;

    int debug_verbosity = 1;  // currently not used
};

#endif
