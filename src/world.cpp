#include "world.h"


world::world(const shared_ptr<softRobots>& soft_robots,
             const shared_ptr<forceContainer>& forces,
             const simParams& sim_params) :
             soft_robots(soft_robots), forces(forces),
             time_step(0), curr_time(0.0), total_time(sim_params.sim_time) {
    // Declare inner elastic forces. These should never be optional.
    forces->addForce(make_shared<elasticStretchingForce>(soft_robots));
    forces->addForce(make_shared<elasticBendingForce>(soft_robots));
    forces->addForce(make_shared<elasticTwistingForce>(soft_robots));

    // Declare inertial force. Should be avoided for explicit methods
    if (sim_params.integrator != FORWARD_EULER && sim_params.integrator != VERLET_POSITION) {
        forces->addForce(make_shared<inertialForce>(soft_robots));
    }

    // Set up the time stepper
    switch(sim_params.integrator) {
        case FORWARD_EULER:
            stepper = make_shared<forwardEuler>(soft_robots, forces, sim_params);
            break;
        case VERLET_POSITION:
            stepper = make_shared<verletPosition>(soft_robots, forces, sim_params);
            break;
        case BACKWARD_EULER:
            stepper = make_shared<backwardEuler>(soft_robots, forces, sim_params, PARDISO_SOLVER);
            break;
        case IMPLICIT_MIDPOINT:
            stepper = make_shared<implicitMidpoint>(soft_robots, forces, sim_params, PARDISO_SOLVER);
            break;
    }

    stepper->initStepper();

    if (sim_params.enable_2d_sim) {
        for (const auto& limb : soft_robots->limbs) limb->enable2DSim();
    }

    // Update boundary conditions
    updateCons();

    // Allocate every thing to prepare for the first iteration
    stepper->updateSystemForNextTimeStep();
}


world::~world() = default;


void world::updateCons()
{
    for (const auto &limb : soft_robots->limbs)
        limb->updateMap();
    stepper->update();
}


void world::updateTimeStep() {
    curr_time += stepper->stepForwardInTime();
    time_step++;
}


void world::printSimData()
{
    auto cf = forces->cf;
    auto ff = forces->ff;
    if (cf && ff) {
        if (cf->getNumCollisions() > 0) {
            printf("time: %.4f | iters: %i | con: %i | min_dist: %.6f | floor_con: %i | f_min_dist: %.6f\n",
                   curr_time, stepper->iter, cf->getNumCollisions(), cf->getMinDist(), ff->num_contacts, ff->min_dist);
        }
        else {
            printf("time: %.4f | iters: %i | con: %i | min_dist: %s | floor_con: %i | f_min_dist: %.6f\n",
                   curr_time, stepper->iter, 0, "N/A", ff->num_contacts, ff->min_dist);
        }
    }
    else if (cf) {
        if (cf->getNumCollisions() > 0) {
            printf("time: %.4f | iters: %i | con: %i | min_dist: %.6f\n",
                   curr_time, stepper->iter, cf->getNumCollisions(), cf->getMinDist());
        }
        else {
            printf("time: %.4f | iters: %i | con: %i | min_dist: %s\n",
                   curr_time, stepper->iter, 0, "N/A");
        }
    }
    else if (ff) {
        printf("time: %.4f | iters: %i | floor_con: %i | f_min_dist: %.6f\n",
               curr_time, stepper->iter, ff->num_contacts, ff->min_dist);
    }
    else {
        printf("time: %.4f | iters: %i\n",
               curr_time, stepper->iter);
    }
}


bool world::simulationRunning() const {
    if (curr_time < total_time)
        return true;
    else
    {
        cout << "Completed simulation." << endl;
        return false;
    }
}


double world::getCoordinate(int i, int limb_idx)
{
    return soft_robots->limbs[limb_idx]->x[i];
}


VectorXd world::getM1(int i, int limb_idx)
{
    return soft_robots->limbs[limb_idx]->m1.row(i);
}


VectorXd world::getM2(int i, int limb_idx)
{
    return soft_robots->limbs[limb_idx]->m2.row(i);
}


int world::getTimeStep() const
{
    return time_step;
}


double world::getCurrentTime() const
{
    return curr_time;
}


bool world::floorExists()
{
    return forces->ff != nullptr;
}

double world::getFloorZ()
{
    if (forces->ff)
        return forces->ff->floor_z;
    throw std::runtime_error("Floor does not exist.");
}
