#ifndef SYMBOLICEQUATIONS_H
#define SYMBOLICEQUATIONS_H

#include <symengine/llvm_double.h>
#include "../eigenIncludes.h"

using namespace SymEngine;


class symbolicEquations
{
public:
    symbolicEquations();

    void generateContactPotentialPiecewiseFunctions();
    void generateFrictionJacobianPiecewiseFunctions();
    void generateFloorContactForce();
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
    bool symbolic_cse;
    int opt_level;

    // Helper functions for symbolic differentiation process
    void subtract_matrix(const DenseMatrix &A, const DenseMatrix &B, DenseMatrix &C);
    void get_norm(const DenseMatrix &num, RCP<const Basic> &C);
    void convert_to_unit_vector(const DenseMatrix &num, DenseMatrix &C);

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

#endif