#include "implicit_time_stepper.h"
#include "rod_mechanics/elastic_joint.h"
#include "rod_mechanics/elastic_rod.h"
#include "rod_mechanics/soft_robots.h"
#include "solvers/dgbsv_solver.h"
#include "solvers/pardiso_solver.h"

ImplicitTimeStepper::ImplicitTimeStepper(const std::shared_ptr<SoftRobots>& soft_robots,
                                         const std::shared_ptr<ForceContainer>& forces,
                                         const SimParams& sim_params, SolverType solver_type)
    : BaseTimeStepper(soft_robots, forces, sim_params), ftol(sim_params.ftol),
      dtol(sim_params.dtol), max_iter(sim_params.max_iter.num_iters),
      terminate_at_max(sim_params.max_iter.terminate_at_max),
      line_search_type(sim_params.line_search), orig_dt(sim_params.dt),
      adaptive_time_stepping_threshold(sim_params.adaptive_time_stepping),
      adaptive_time_stepping(sim_params.adaptive_time_stepping != 0), solver_type(solver_type) {
}

ImplicitTimeStepper::~ImplicitTimeStepper() = default;

void ImplicitTimeStepper::initStepper() {
    BaseTimeStepper::initStepper();
    switch (solver_type) {
        case PARDISO_SOLVER:
            solver = std::make_unique<PardisoSolver>(shared_from_this());
            break;

        case DGBSV_SOLVER:
            throw std::runtime_error("DGBSV_SOLVER is deprecated.");
            solver = std::make_unique<DGBSVSolver>(shared_from_this());
            DGBSVSolver local_solver = dynamic_cast<DGBSVSolver&>(*solver);
            dgbsv_jacobian_len = local_solver.NUMROWS * freeDOF;
            kl = local_solver.kl;
            ku = local_solver.ku;
            num_rows = local_solver.NUMROWS;
            dgbsv_jacobian = std::vector<double>(dgbsv_jacobian_len, 0);
            break;
    }
}

std::shared_ptr<ImplicitTimeStepper> ImplicitTimeStepper::shared_from_this() {
    return std::static_pointer_cast<ImplicitTimeStepper>(BaseTimeStepper::shared_from_this());
}

void ImplicitTimeStepper::integrator() {
    solver->integrator();
}

template <>
void ImplicitTimeStepper::addJacobian<PARDISO_SOLVER>(int ind1, int ind2, double p, int limb_idx1,
                                                      int limb_idx2) {
    auto limb1 = limbs[limb_idx1];
    auto limb2 = limbs[limb_idx2];
    if (limb1->getIfConstrained(ind1) == 0 && limb2->getIfConstrained(ind2) == 0) {
        int mappedInd1 = limb1->fullToUnconsMap[ind1];
        int mappedInd2 = limb2->fullToUnconsMap[ind2];

        int offset1 = offsets[limb_idx1];
        int offset2 = offsets[limb_idx2];

        int jac_ind1 = mappedInd2 + offset2;
        int jac_ind2 = mappedInd1 + offset1;

        jacobian_triplets.emplace_back(jac_ind1, jac_ind2, p);
    }
}

template <>
void ImplicitTimeStepper::addJacobian<DGBSV_SOLVER>(int ind1, int ind2, double p, int limb_idx1,
                                                    int limb_idx2) {
    throw std::runtime_error("DGBSV_SOLVER is deprecated.");
    //    std::shared_ptr<ElasticRod> limb1 = limbs[limb_idx1];
    //    std::shared_ptr<ElasticRod> limb2 = limbs[limb_idx2];
    //    mappedInd1 = limb1->fullToUnconsMap[ind1];
    //    mappedInd2 = limb2->fullToUnconsMap[ind2];
    //    int offset1 = offsets[limb_idx1];
    //    int offset2 = offsets[limb_idx2];
    //    int jac_ind1, jac_ind2;
    //
    //    //    DGBSVSolver ls = dynamic_cast<DGBSVSolver&>(*solver);
    //
    //    int dgbsv_row, dgbsv_col, dgbsv_offset;
    //
    //    if (limb1->getIfConstrained(ind1) == 0 && limb2->getIfConstrained(ind2) == 0) {
    //        jac_ind1 = mappedInd2 + offset2;
    //        jac_ind2 = mappedInd1 + offset1;
    //
    //        //        dgbsv_row = ls.kl + ls.ku + jac_ind1 - jac_ind2;
    //        //        dgbsv_col = jac_ind2;
    //        //        dgbsv_offset = dgbsv_row + dgbsv_col * ls.NUMROWS;
    //
    //        dgbsv_row = kl + ku + jac_ind1 - jac_ind2;
    //        dgbsv_col = jac_ind2;
    //        dgbsv_offset = dgbsv_row + dgbsv_col * num_rows;
    //
    //        dgbsv_jacobian[dgbsv_offset] += p;
    //        Jacobian(jac_ind1, jac_ind2) += p;
    //    }
}

void ImplicitTimeStepper::addJacobian(int ind1, int ind2, double p, int limb_idx) {
    addJacobian(ind1, ind2, p, limb_idx, limb_idx);
}

void ImplicitTimeStepper::addJacobian(int ind1, int ind2, double p, int limb_idx1, int limb_idx2) {
    switch (solver_type) {
        case PARDISO_SOLVER:
            addJacobian<PARDISO_SOLVER>(ind1, ind2, p, limb_idx1, limb_idx2);
            break;
        case DGBSV_SOLVER:
            addJacobian<DGBSV_SOLVER>(ind1, ind2, p, limb_idx1, limb_idx2);
            break;
    }
}

void ImplicitTimeStepper::setZero() {
    BaseTimeStepper::setZero();

    switch (solver_type) {
        case PARDISO_SOLVER:
            break;
        case DGBSV_SOLVER:
            for (int i = 0; i < dgbsv_jacobian_len; i++)
                dgbsv_jacobian[i] = 0;
            break;
    }
    jacobian_triplets.clear();
}

void ImplicitTimeStepper::update() {
    BaseTimeStepper::update();

    switch (solver_type) {
        case PARDISO_SOLVER:
            break;
        case DGBSV_SOLVER:
            DGBSVSolver local_solver = dynamic_cast<DGBSVSolver&>(*solver);
            dgbsv_jacobian_len = local_solver.NUMROWS * freeDOF;
            dgbsv_jacobian.clear();
            dgbsv_jacobian.resize(dgbsv_jacobian_len, 0);
            break;
    }
    jacobian_triplets.clear();
}

void ImplicitTimeStepper::prepSystemForIteration() {
    BaseTimeStepper::prepSystemForIteration();
    ImplicitTimeStepper::setZero();
}
