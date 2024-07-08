#include "implicitTimeStepper.h"
#include "solvers/pardisoSolver.h"
#include "solvers/dgbsvSolver.h"
#include <algorithm>


implicitTimeStepper::implicitTimeStepper(const shared_ptr<softRobots>& soft_robots,
                                         const shared_ptr<forceContainer>& forces,
                                         const simParams& sim_params,
                                         solverType solver_type) :
                                         baseTimeStepper(soft_robots, forces, sim_params),
                                         ftol(sim_params.ftol), dtol(sim_params.dtol
                                         ), max_iter(sim_params.max_iter.num_iters),
                                         terminate_at_max(sim_params.max_iter.terminate_at_max),
                                         line_search(sim_params.line_search), orig_dt(sim_params.dt),
                                         adaptive_time_stepping_threshold(sim_params.adaptive_time_stepping),
                                         adaptive_time_stepping(sim_params.adaptive_time_stepping != 0),
                                         solver_type(solver_type)
{
    Jacobian = MatrixXd::Zero(freeDOF, freeDOF);
    line_search_type = (sim_params.line_search_type == Golden)? Golden:Wolfe;
}


implicitTimeStepper::~implicitTimeStepper() {
    delete [] dgbsv_jacobian;
    delete [] ia;
}


void implicitTimeStepper::initStepper() {
    baseTimeStepper::initStepper();
    switch (solver_type) {
        case PARDISO_SOLVER:
            solver = make_unique<pardisoSolver>(shared_from_this());
            ia = new MKL_INT[freeDOF+1]{0};
            ia[0] = 1;
            break;

        case DGBSV_SOLVER:
            solver = make_unique<dgbsvSolver>(shared_from_this());
            dgbsvSolver local_solver = dynamic_cast<dgbsvSolver&>(*solver);
            dgbsv_jacobian_len = local_solver.NUMROWS * freeDOF;
            kl = local_solver.kl;
            ku = local_solver.ku;
            num_rows = local_solver.NUMROWS;
            dgbsv_jacobian = new double[dgbsv_jacobian_len]{0};
            break;
    }
}


shared_ptr<implicitTimeStepper> implicitTimeStepper::shared_from_this() {
    return static_pointer_cast<implicitTimeStepper>(baseTimeStepper::shared_from_this());
}


void implicitTimeStepper::integrator() {
    solver->integrator();
}


template<>
void implicitTimeStepper::addJacobian<PARDISO_SOLVER>(int ind1, int ind2, double p, int limb_idx1, int limb_idx2) {
    shared_ptr<elasticRod> limb1 = limbs[limb_idx1];
    shared_ptr<elasticRod> limb2 = limbs[limb_idx2];
    mappedInd1 = limb1->fullToUnconsMap[ind1];
    mappedInd2 = limb2->fullToUnconsMap[ind2];
    int offset1 = offsets[limb_idx1];
    int offset2 = offsets[limb_idx2];
    int jac_ind1, jac_ind2;
    if (limb1->getIfConstrained(ind1) == 0 && limb2->getIfConstrained(ind2) == 0) {
        jac_ind1 = mappedInd2+offset2;
        jac_ind2 = mappedInd1+offset1;
        if (Jacobian(jac_ind1, jac_ind2) == 0 && p != 0) {
            ia[jac_ind1+1]++;
            non_zero_elements.emplace_back(jac_ind1, jac_ind2);
        }
        else if (Jacobian(jac_ind1, jac_ind2) != 0 && Jacobian(jac_ind1, jac_ind2) + p == 0) {
            ia[jac_ind1+1]--;
            // This is expensive, but it doesn't happen that often. Better than using a set
            non_zero_elements.erase(remove(non_zero_elements.begin(),
                                           non_zero_elements.end(),
                                           pair<int, int>(jac_ind1, jac_ind2)), non_zero_elements.end());
        }
        Jacobian(jac_ind1, jac_ind2) += p;
    }
}


template<>
void implicitTimeStepper::addJacobian<DGBSV_SOLVER>(int ind1, int ind2, double p, int limb_idx1, int limb_idx2) {
    shared_ptr<elasticRod> limb1 = limbs[limb_idx1];
    shared_ptr<elasticRod> limb2 = limbs[limb_idx2];
    mappedInd1 = limb1->fullToUnconsMap[ind1];
    mappedInd2 = limb2->fullToUnconsMap[ind2];
    int offset1 = offsets[limb_idx1];
    int offset2 = offsets[limb_idx2];
    int jac_ind1, jac_ind2;

//    dgbsvSolver ls = dynamic_cast<dgbsvSolver&>(*solver);

    int dgbsv_row, dgbsv_col, dgbsv_offset;

    if (limb1->getIfConstrained(ind1) == 0 && limb2->getIfConstrained(ind2) == 0) {
        jac_ind1 = mappedInd2+offset2;
        jac_ind2 = mappedInd1+offset1;

//        dgbsv_row = ls.kl + ls.ku + jac_ind1 - jac_ind2;
//        dgbsv_col = jac_ind2;
//        dgbsv_offset = dgbsv_row + dgbsv_col * ls.NUMROWS;

        dgbsv_row = kl + ku + jac_ind1 - jac_ind2;
        dgbsv_col = jac_ind2;
        dgbsv_offset = dgbsv_row + dgbsv_col * num_rows;

        dgbsv_jacobian[dgbsv_offset] += p;
        Jacobian(jac_ind1, jac_ind2) += p;
    }
}


void implicitTimeStepper::addJacobian(int ind1, int ind2, double p, int limb_idx) {
    addJacobian(ind1, ind2, p, limb_idx, limb_idx);
}


void implicitTimeStepper::addJacobian(int ind1, int ind2, double p, int limb_idx1, int limb_idx2) {
    switch (solver_type) {
        case PARDISO_SOLVER:
            addJacobian<PARDISO_SOLVER>(ind1, ind2, p, limb_idx1, limb_idx2);
            break;
        case DGBSV_SOLVER:
            addJacobian<DGBSV_SOLVER>(ind1, ind2, p, limb_idx1, limb_idx2);
            break;
    }
}


void implicitTimeStepper::setZero() {
    baseTimeStepper::setZero();

    switch (solver_type) {
        case PARDISO_SOLVER:
            non_zero_elements.clear();
            for (int i = 0; i < freeDOF; i++)
                ia[i+1] = 0;
            break;
        case DGBSV_SOLVER:
            for (int i=0; i < dgbsv_jacobian_len; i++)
                dgbsv_jacobian[i] = 0;
            break;
    }
    Jacobian = MatrixXd::Zero(freeDOF,freeDOF);
}


void implicitTimeStepper::update() {
    baseTimeStepper::update();

    switch (solver_type) {
        case PARDISO_SOLVER:
            delete [] ia;
            ia = new MKL_INT[freeDOF+1]{0};
            ia[0] = 1;
            break;
        case DGBSV_SOLVER:
            delete [] dgbsv_jacobian;
            dgbsvSolver local_solver = dynamic_cast<dgbsvSolver&>(*solver);
            dgbsv_jacobian_len = local_solver.NUMROWS * freeDOF;
            dgbsv_jacobian = new double[dgbsv_jacobian_len]{0};
            break;
    }
    Jacobian = MatrixXd::Zero(freeDOF,freeDOF);
}


void implicitTimeStepper::prepSystemForIteration() {
    baseTimeStepper::prepSystemForIteration();
    implicitTimeStepper::setZero();
}
