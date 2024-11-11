#ifndef SYMBOLIC_EQUATIONS_H
#define SYMBOLIC_EQUATIONS_H

#include "global_definitions.h"
#include <symengine/llvm_double.h>

using namespace SymEngine;

class SymbolicEquations
{
  public:
    SymbolicEquations();

    void generateContactPotentialPiecewiseFunctions();
    void generateFrictionJacobianPiecewiseFunctions();
    //    void generateFloorContactForce();
    void generateFloorFrictionJacobianFunctions();

    LLVMDoubleVisitor E_p2p_gradient_func;
    LLVMDoubleVisitor E_p2p_hessian_func;
    LLVMDoubleVisitor E_e2p_gradient_func;
    LLVMDoubleVisitor E_e2p_hessian_func;
    LLVMDoubleVisitor E_e2e_gradient_func;
    LLVMDoubleVisitor E_e2e_hessian_func;

    LLVMDoubleVisitor E_p2p_pen_gradient_func;
    LLVMDoubleVisitor E_p2p_pen_hessian_func;
    LLVMDoubleVisitor E_e2p_pen_gradient_func;
    LLVMDoubleVisitor E_e2p_pen_hessian_func;
    LLVMDoubleVisitor E_e2e_pen_gradient_func;
    LLVMDoubleVisitor E_e2e_pen_hessian_func;

    LLVMDoubleVisitor friction_partials_dfr_dx_sticking_func;
    LLVMDoubleVisitor friction_partials_dfr_dfc_sticking_func;
    LLVMDoubleVisitor friction_partials_dfr_dx_sliding_func;
    LLVMDoubleVisitor friction_partials_dfr_dfc_sliding_func;

    LLVMDoubleVisitor floor_friction_partials_dfr_dx_func;
    LLVMDoubleVisitor floor_friction_partials_dfr_dfn_func;
    LLVMDoubleVisitor floor_friction_partials_gamma1_dfr_dx_func;
    LLVMDoubleVisitor floor_friction_partials_gamma1_dfr_dfn_func;

  private:
    // Helper functions for symbolic differentiation process
    static void subtractMatrix(const DenseMatrix& A, const DenseMatrix& B, DenseMatrix& C);
    static void getNorm(const DenseMatrix& num, RCP<const Basic>& C);
    static void convertToUnitVector(const DenseMatrix& num, DenseMatrix& C);

    bool symbolic_cse;
    int opt_level;

    RCP<const Basic> x1s_x;
    RCP<const Basic> x1s_y;
    RCP<const Basic> x1s_z;
    RCP<const Basic> x1e_x;
    RCP<const Basic> x1e_y;
    RCP<const Basic> x1e_z;
    RCP<const Basic> x2s_x;
    RCP<const Basic> x2s_y;
    RCP<const Basic> x2s_z;
    RCP<const Basic> x2e_x;
    RCP<const Basic> x2e_y;
    RCP<const Basic> x2e_z;
    RCP<const Basic> K1;
    RCP<const Basic> h2;

    RCP<const Basic> x1s_x0;
    RCP<const Basic> x1s_y0;
    RCP<const Basic> x1s_z0;
    RCP<const Basic> x1e_x0;
    RCP<const Basic> x1e_y0;
    RCP<const Basic> x1e_z0;
    RCP<const Basic> x2s_x0;
    RCP<const Basic> x2s_y0;
    RCP<const Basic> x2s_z0;
    RCP<const Basic> x2e_x0;
    RCP<const Basic> x2e_y0;
    RCP<const Basic> x2e_z0;
    RCP<const Basic> f1s_x;
    RCP<const Basic> f1s_y;
    RCP<const Basic> f1s_z;
    RCP<const Basic> f1e_x;
    RCP<const Basic> f1e_y;
    RCP<const Basic> f1e_z;
    RCP<const Basic> f2s_x;
    RCP<const Basic> f2s_y;
    RCP<const Basic> f2s_z;
    RCP<const Basic> f2e_x;
    RCP<const Basic> f2e_y;
    RCP<const Basic> f2e_z;
    RCP<const Basic> mu;
    RCP<const Basic> dt;
    RCP<const Basic> K2;

    // Floor contact variables
    RCP<const Basic> z;
    RCP<const Basic> floor_z;
};

#endif  // SYMBOLIC_EQUATIONS_H
