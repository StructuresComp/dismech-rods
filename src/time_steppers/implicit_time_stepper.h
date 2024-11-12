#ifndef IMPLICIT_TIME_STEPPER_H
#define IMPLICIT_TIME_STEPPER_H

#include "base_time_stepper.h"
#include "solvers/solver_types.h"

class BaseSolver;

class ImplicitTimeStepper : public BaseTimeStepper
{
  public:
    ImplicitTimeStepper(const std::shared_ptr<SoftRobots>& soft_robots,
                        const std::shared_ptr<ForceContainer>& forces, const SimParams& sim_params,
                        SolverType solver_type);
    ~ImplicitTimeStepper() override;

    void addJacobian(int ind1, int ind2, double p, int limb_idx) override;
    void addJacobian(int ind1, int ind2, double p, int limb_idx1, int limb_idx2) override;
    void setZero() override;
    void update() override;
    void integrator() override;
    void initStepper() override;

    void prepSystemForIteration() override;

    // Utility variables for PARDISO solver.
    // Need to keep track of non-zero elements
    // in ImplicitTimeStepper to avoid n^2 construction later.
    // This allows us to keep complexity to nlogn.
    int* ia{};
    std::vector<std::pair<int, int>> non_zero_elements;

    // For dgbsv solver
    double* dgbsv_jacobian{};
    int kl{}, ku{}, num_rows{};

  protected:
    virtual double newtonMethod(double dt) = 0;
    virtual double lineSearch(double dt) = 0;

    double ftol;
    double dtol;
    int max_iter;
    bool terminate_at_max;
    double orig_dt;
    bool adaptive_time_stepping;
    int adaptive_time_stepping_threshold;
    LineSearchType line_search_type;

    template <SolverType solver_type>
    void addJacobian(int ind1, int ind2, double p, int limb_idx1, int limb_idx2);

  private:
    std::shared_ptr<ImplicitTimeStepper> shared_from_this();  // NOLINT
    std::unique_ptr<BaseSolver> solver;
    SolverType solver_type;
    int dgbsv_jacobian_len{};
};

#endif  // IMPLICIT_TIME_STEPPER_H
