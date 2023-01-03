#include "symbolicEquations.h"


symbolicEquations::symbolicEquations() {
    x1s_x = symbol("x1s_x");
    x1s_y = symbol("x1s_y");
    x1s_z = symbol("x1s_z");
    x1e_x = symbol("x1e_x");
    x1e_y = symbol("x1e_y");
    x1e_z = symbol("x1e_z");
    x2s_x = symbol("x2s_x");
    x2s_y = symbol("x2s_y");
    x2s_z = symbol("x2s_z");
    x2e_x = symbol("x2e_x");
    x2e_y = symbol("x2e_y");
    x2e_z = symbol("x2e_z");
    K1 = symbol("K1");
    h2 = symbol("h2");

    x1s_x0 = symbol("x1s_x0");
    x1s_y0 = symbol("x1s_y0");
    x1s_z0 = symbol("x1s_z0");
    x1e_x0 = symbol("x1e_x0");
    x1e_y0 = symbol("x1e_y0");
    x1e_z0 = symbol("x1e_z0");
    x2s_x0 = symbol("x2s_x0");
    x2s_y0 = symbol("x2s_y0");
    x2s_z0 = symbol("x2s_z0");
    x2e_x0 = symbol("x2e_x0");
    x2e_y0 = symbol("x2e_y0");
    x2e_z0 = symbol("x2e_z0");
    f1s_x = symbol("f1s_x");
    f1s_y = symbol("f1s_y");
    f1s_z = symbol("f1s_z");
    f1e_x = symbol("f1e_x");
    f1e_y = symbol("f1e_y");
    f1e_z = symbol("f1e_z");
    f2s_x = symbol("f2s_x");
    f2s_y = symbol("f2s_y");
    f2s_z = symbol("f2s_z");
    f2e_x = symbol("f2e_x");
    f2e_y = symbol("f2e_y");
    f2e_z = symbol("f2e_z");
    mu = symbol("mu");
    dt = symbol("dt");
    K2 = symbol("K2");

    symbolic_cse = true;
    opt_level = 3;
}


// For some reason SymEngine doesn't have this implemented X_X
void symbolicEquations::subtract_matrix(const DenseMatrix &A, const DenseMatrix &B, DenseMatrix &C) {
    assert((A.nrows() == B.nrows()) && (A.ncols() == B.ncols()));
    for (unsigned i=0; i < A.nrows(); i++) {
        for (unsigned j=0; j < A.ncols(); j++) {
            C.set(i, j, sub(A.get(i, j), B.get(i, j)));
        }
    }
}



void symbolicEquations::get_norm(const DenseMatrix &num, RCP<const Basic> &C) {
    DenseMatrix tmp(num.nrows(), num.ncols());
    num.elementwise_mul_matrix(num, tmp);
    C = sqrt(add(tmp.as_vec_basic()));
}


void symbolicEquations::convert_to_unit_vector(const DenseMatrix &num, DenseMatrix &C) {
    DenseMatrix tmp(num.nrows(), num.ncols());
    num.elementwise_mul_matrix(num, tmp);
    auto norm = sqrt(add(tmp.as_vec_basic()));
    for (unsigned i=0; i < num.nrows(); i++) {
        for (unsigned j=0; j < num.ncols(); j++) {
            C.set(i, j, div(num.get(i, j), norm));
        }
    }
}



void symbolicEquations::generateContactPotentialPiecewiseFunctions() {
    // POINT TO POINT
    DenseMatrix x1s({x1s_x, x1s_y, x1s_z});
    DenseMatrix x1e({x1e_x, x1e_y, x1e_z});
    DenseMatrix x2s({x2s_x, x2s_y, x2s_z});
    DenseMatrix x2e({x2e_x, x2e_y, x2e_z});
    int num_rows = x1s.nrows();
    int num_cols = x1s.ncols();

    vec_basic nodes_vec_p2p {x1s_x, x1s_y, x1s_z,
                             x1e_x, x1e_y, x1e_z};
    DenseMatrix nodes_p2p {nodes_vec_p2p};

    vec_basic func_p2p_inputs(nodes_vec_p2p);
    func_p2p_inputs.push_back(K1);
    func_p2p_inputs.push_back(h2);

    DenseMatrix e1(num_rows, num_cols);
    subtract_matrix(x1e, x1s, e1);
    RCP<const Basic> dist_p2p;
    get_norm(e1, dist_p2p);

    // Point to Point   2h - δ < Δ < 2h + δ
    RCP<const Basic> E_p2p = pow(mul(div(one, K1), log(add(one, exp(mul(K1, sub(h2, dist_p2p)))))), 2);
    DenseMatrix E_p2p_potential({E_p2p});

    DenseMatrix E_p2p_gradient(1, 6);
    jacobian(E_p2p_potential, nodes_p2p, E_p2p_gradient);
    DenseMatrix E_p2p_hessian(6, 6);
    jacobian(E_p2p_gradient, nodes_p2p, E_p2p_hessian);

    E_p2p_gradient_func.init(func_p2p_inputs, E_p2p_gradient.as_vec_basic(), symbolic_cse, opt_level);
    E_p2p_hessian_func.init(func_p2p_inputs, E_p2p_hessian.as_vec_basic(), symbolic_cse, opt_level);

    // Point to Point    Δ <= 2h - δ
    RCP<const Basic> E_p2p_pen = pow(sub(h2, dist_p2p), 2);
    DenseMatrix E_p2p_pen_potential{{E_p2p_pen}};

    DenseMatrix E_p2p_pen_gradient(1, 6);
    jacobian(E_p2p_pen_potential, nodes_p2p, E_p2p_pen_gradient);
    DenseMatrix E_p2p_pen_hessian(6, 6);
    jacobian(E_p2p_pen_gradient, nodes_p2p, E_p2p_pen_hessian);

    E_p2p_pen_gradient_func.init(func_p2p_inputs, E_p2p_pen_gradient.as_vec_basic(), symbolic_cse, opt_level);
    E_p2p_pen_hessian_func.init(func_p2p_inputs, E_p2p_pen_hessian.as_vec_basic(), symbolic_cse, opt_level);

    // POINT TO EDGE
    vec_basic nodes_vec_e2p {x1s_x, x1s_y, x1s_z,
                             x1e_x, x1e_y, x1e_z,
                             x2s_x, x2s_y, x2s_z};
    DenseMatrix nodes_e2p {nodes_vec_e2p};
    vec_basic func_e2p_inputs(nodes_vec_e2p);
    func_e2p_inputs.push_back(K1);
    func_e2p_inputs.push_back(h2);

    DenseMatrix e13(num_rows, num_cols);
    subtract_matrix(x1s, x2s, e13);
    DenseMatrix e23(num_rows, num_cols);
    subtract_matrix(x1e, x2s, e23);
    DenseMatrix temp(num_rows, num_cols);
    cross(e13, e23, temp);

    RCP<const Basic> frac1;
    get_norm(temp, frac1);

    RCP<const Basic> dist_e2p = div(frac1, dist_p2p);

    // Point to Edge   2h - δ < Δ < 2h + δ
    RCP<const Basic> E_e2p = pow(mul(div(one, K1), log(add(one, exp(mul(K1, sub(h2, dist_e2p)))))), 2);
    DenseMatrix E_e2p_potential{{E_e2p}};

    DenseMatrix E_e2p_gradient(1, 9);
    jacobian(E_e2p_potential, nodes_e2p, E_e2p_gradient);
    DenseMatrix E_e2p_hessian(9, 9);
    jacobian(E_e2p_gradient, nodes_e2p, E_e2p_hessian);

    E_e2p_gradient_func.init(func_e2p_inputs, E_e2p_gradient.as_vec_basic(), symbolic_cse, opt_level);
    E_e2p_hessian_func.init(func_e2p_inputs, E_e2p_hessian.as_vec_basic(), symbolic_cse, opt_level);

    // Point to Edge    Δ <= 2h - δ
    RCP<const Basic> E_e2p_pen = pow(sub(h2, dist_e2p), 2);
    DenseMatrix E_e2p_pen_potential{{E_e2p_pen}};

    DenseMatrix E_e2p_pen_gradient(1, 9);
    jacobian(E_e2p_pen_potential, nodes_e2p, E_e2p_pen_gradient);
    DenseMatrix E_e2p_pen_hessian(9, 9);
    jacobian(E_e2p_pen_gradient, nodes_e2p, E_e2p_pen_hessian);

    E_e2p_pen_gradient_func.init(func_e2p_inputs, E_e2p_pen_gradient.as_vec_basic(), symbolic_cse, opt_level);
    E_e2p_pen_hessian_func.init(func_e2p_inputs, E_e2p_pen_hessian.as_vec_basic(), symbolic_cse, opt_level);

    // EDGE TO EDGE
    vec_basic nodes_vec_e2e {x1s_x, x1s_y, x1s_z,
                             x1e_x, x1e_y, x1e_z,
                             x2s_x, x2s_y, x2s_z,
                             x2e_x, x2e_y, x2e_z};
    DenseMatrix nodes_e2e {nodes_vec_e2e};
    vec_basic func_e2e_inputs(nodes_vec_e2e);
    func_e2e_inputs.push_back(K1);
    func_e2e_inputs.push_back(h2);

    DenseMatrix e2(num_rows, num_cols);
    subtract_matrix(x2e, x2s, e2);
    DenseMatrix num(num_rows, num_cols);
    cross(e1, e2, num);
    DenseMatrix num_hat(num_rows, num_cols);
    convert_to_unit_vector(num, num_hat);
    num_hat.elementwise_mul_matrix(e13, temp);
    RCP<const Basic> dist_e2e = sqrt(pow(add(temp.as_vec_basic()), 2));

    // Edge to Edge   2h - δ < Δ < 2h + δ
    RCP<const Basic> E_e2e = pow(mul(div(one, K1), log(add(one, exp(mul(K1, sub(h2, dist_e2e)))))), 2);
    DenseMatrix E_e2e_potential{{E_e2e}};

    DenseMatrix E_e2e_gradient(1, 12);
    jacobian(E_e2e_potential, nodes_e2e, E_e2e_gradient);
    DenseMatrix E_e2e_hessian(12, 12);
    jacobian(E_e2e_gradient, nodes_e2e, E_e2e_hessian);

    E_e2e_gradient_func.init(func_e2e_inputs, E_e2e_gradient.as_vec_basic(), symbolic_cse, opt_level);
    E_e2e_hessian_func.init(func_e2e_inputs, E_e2e_hessian.as_vec_basic(), symbolic_cse, opt_level);

    // Edge to Edge    Δ <= 2h - δ
    RCP<const Basic> E_e2e_pen = pow(sub(h2, dist_e2e), 2);
    DenseMatrix E_e2e_pen_potential{{E_e2e_pen}};

    DenseMatrix E_e2e_pen_gradient(1, 12);
    jacobian(E_e2e_pen_potential, nodes_e2e, E_e2e_pen_gradient);
    DenseMatrix E_e2e_pen_hessian(12, 12);
    jacobian(E_e2e_pen_gradient, nodes_e2e, E_e2e_pen_hessian);

    E_e2e_pen_gradient_func.init(func_e2e_inputs, E_e2e_pen_gradient.as_vec_basic(), symbolic_cse, opt_level);
    E_e2e_pen_hessian_func.init(func_e2e_inputs, E_e2e_pen_hessian.as_vec_basic(), symbolic_cse, opt_level);
}


void symbolicEquations::generateFrictionJacobianPiecewiseFunctions() {
    DenseMatrix nodes{{x1s_x, x1s_y, x1s_z,
                       x1e_x, x1e_y, x1e_z,
                       x2s_x, x2s_y, x2s_z,
                       x2e_x, x2e_y, x2e_z}};

    // Construct Symbolic Arrays for each node
    DenseMatrix x1s({x1s_x, x1s_y, x1s_z});
    DenseMatrix x1e({x1e_x, x1e_y, x1e_z});
    DenseMatrix x2s({x2s_x, x2s_y, x2s_z});
    DenseMatrix x2e({x2e_x, x2e_y, x2e_z});
    DenseMatrix x1s_0({x1s_x0, x1s_y0, x1s_z0});
    DenseMatrix x1e_0({x1e_x0, x1e_y0, x1e_z0});
    DenseMatrix x2s_0({x2s_x0, x2s_y0, x2s_z0});
    DenseMatrix x2e_0({x2e_x0, x2e_y0, x2e_z0});
    DenseMatrix f1s({f1s_x, f1s_y, f1s_z});
    DenseMatrix f1e({f1e_x, f1e_y, f1e_z});
    DenseMatrix f2s({f2s_x, f2s_y, f2s_z});
    DenseMatrix f2e({f2e_x, f2e_y, f2e_z});

    vec_basic ffr_input {x1s_x, x1s_y, x1s_z,
                         x1e_x, x1e_y, x1e_z,
                         x2s_x, x2s_y, x2s_z,
                         x2e_x, x2e_y, x2e_z,
                         x1s_x0, x1s_y0, x1s_z0,
                         x1e_x0, x1e_y0, x1e_z0,
                         x2s_x0, x2s_y0, x2s_z0,
                         x2e_x0, x2e_y0, x2e_z0,
                         f1s_x, f1s_y, f1s_z,
                         f1e_x, f1e_y, f1e_z,
                         f2s_x, f2s_y, f2s_z,
                         f2e_x, f2e_y, f2e_z,
                         mu, dt, K2};

    vec_basic cforces {f1s_x, f1s_y, f1s_z,
                       f1e_x, f1e_y, f1e_z,
                       f2s_x, f2s_y, f2s_z,
                       f2e_x, f2e_y, f2e_z};

    DenseMatrix f1(3, 1);
    DenseMatrix f2(3, 1);
    f1s.add_matrix(f1e, f1);
    f2s.add_matrix(f2e, f2);

    RCP<const Basic> f1s_n;
    get_norm(f1s, f1s_n);
    RCP<const Basic> f1e_n;
    get_norm(f1e, f1e_n);
    RCP<const Basic> f2s_n;
    get_norm(f2s, f2s_n);
    RCP<const Basic> f2e_n;
    get_norm(f2e, f2e_n);
    RCP<const Basic> f1_n;
    get_norm(f1, f1_n);
    RCP<const Basic> f2_n;
    get_norm(f2, f2_n);

    RCP<const Basic> beta11 = div(f1s_n, f1_n);
    RCP<const Basic> beta12 = div(f1e_n, f1_n);
    RCP<const Basic> beta21 = div(f2s_n, f2_n);
    RCP<const Basic> beta22 = div(f2e_n, f2_n);

    DenseMatrix contact_norm(3, 1);
    convert_to_unit_vector(f1, contact_norm);

    DenseMatrix v1s(3, 1);
    DenseMatrix v1e(3, 1);
    DenseMatrix v2s(3, 1);
    DenseMatrix v2e(3, 1);
    subtract_matrix(x1s, x1s_0, v1s);
    subtract_matrix(x1e, x1e_0, v1e);
    subtract_matrix(x2s, x2s_0, v2s);
    subtract_matrix(x2e, x2e_0, v2e);
    v1s.mul_scalar(div(one, dt), v1s);
    v1e.mul_scalar(div(one, dt), v1e);
    v2s.mul_scalar(div(one, dt), v2s);
    v2e.mul_scalar(div(one, dt), v2e);

    DenseMatrix v1s_r(3, 1);
    DenseMatrix v1e_r(3, 1);
    DenseMatrix v2s_r(3, 1);
    DenseMatrix v2e_r(3, 1);
    v1s.mul_scalar(beta11, v1s_r);
    v1e.mul_scalar(beta12, v1e_r);
    v2s.mul_scalar(beta21, v2s_r);
    v2e.mul_scalar(beta22, v2e_r);

    DenseMatrix v1(3, 1);
    DenseMatrix v2(3, 1);
    v1s_r.add_matrix(v1e_r, v1);
    v2s_r.add_matrix(v2e_r, v2);

    DenseMatrix v_rel(3, 1);
    subtract_matrix(v1, v2, v_rel);

    // Compute tangent velocity of edge 1
    DenseMatrix tv_rel(3, 1);
    v_rel.elementwise_mul_matrix(contact_norm, tv_rel);
    RCP<const Basic> tmp = add(tv_rel.as_vec_basic());
    contact_norm.mul_scalar(tmp, tv_rel);
    subtract_matrix(v_rel, tv_rel, tv_rel);

    RCP<const Basic> tv_rel_n;
    get_norm(tv_rel, tv_rel_n);

    DenseMatrix tv_rel_u(3, 1);
    convert_to_unit_vector(tv_rel, tv_rel_u);

    RCP<const Basic> gamma = sub(div(integer(2), add(one, exp(mul(integer(-1), mul(K2, tv_rel_n))))), one);

    // STICKING FRICTION JACOBIAN   0 < γ < 1

    DenseMatrix ffr1(3, 1);
    DenseMatrix ffr2(3, 1);
    tv_rel_u.mul_scalar(mul(gamma, mu), ffr1);
    ffr1.mul_scalar(integer(-1), ffr2);

    DenseMatrix ffr1s(3, 1);
    DenseMatrix ffr1e(3, 1);
    DenseMatrix ffr2s(3, 1);
    DenseMatrix ffr2e(3, 1);
    ffr1.mul_scalar(f1s_n, ffr1s);
    ffr1.mul_scalar(f1e_n, ffr1e);
    ffr2.mul_scalar(f2s_n, ffr2s);
    ffr2.mul_scalar(f2e_n, ffr2e);

    DenseMatrix ffr_vec1({ffr1s.get(0, 0), ffr1s.get(1, 0), ffr1s.get(2, 0),
                          ffr1e.get(0, 0), ffr1e.get(1, 0), ffr1e.get(2, 0),
                          ffr2s.get(0, 0), ffr2s.get(1, 0), ffr2s.get(2, 0),
                          ffr2e.get(0, 0), ffr2e.get(1, 0), ffr2e.get(2, 0)});

    DenseMatrix friction_partial_dfr_dx1(12, 12);
    DenseMatrix friction_partial_dfr_dfc1(12, 12);
    jacobian(ffr_vec1, nodes, friction_partial_dfr_dx1);
    jacobian(ffr_vec1, cforces, friction_partial_dfr_dfc1);

    friction_partials_dfr_dx_sticking_func.init(ffr_input, friction_partial_dfr_dx1.as_vec_basic(), symbolic_cse, opt_level);
    friction_partials_dfr_dfc_sticking_func.init(ffr_input, friction_partial_dfr_dfc1.as_vec_basic(), symbolic_cse, opt_level);

    // SLIDING FRICTION JACOBIAN   γ >= 1

    tv_rel_u.mul_scalar(mu, ffr1);
    ffr1.mul_scalar(integer(-1), ffr2);
    ffr1.mul_scalar(f1s_n, ffr1s);
    ffr1.mul_scalar(f1e_n, ffr1e);
    ffr2.mul_scalar(f2s_n, ffr2s);
    ffr2.mul_scalar(f2e_n, ffr2e);

    DenseMatrix ffr_vec2({ffr1s.get(0, 0), ffr1s.get(1, 0), ffr1s.get(2, 0),
                          ffr1e.get(0, 0), ffr1e.get(1, 0), ffr1e.get(2, 0),
                          ffr2s.get(0, 0), ffr2s.get(1, 0), ffr2s.get(2, 0),
                          ffr2e.get(0, 0), ffr2e.get(1, 0), ffr2e.get(2, 0)});

    DenseMatrix friction_partial_dfr_dx2(12, 12);
    DenseMatrix friction_partial_dfr_dfc2(12, 12);
    jacobian(ffr_vec2, nodes, friction_partial_dfr_dx2);
    jacobian(ffr_vec2, cforces, friction_partial_dfr_dfc2);

    friction_partials_dfr_dx_sliding_func.init(ffr_input, friction_partial_dfr_dx2.as_vec_basic(), symbolic_cse, opt_level);
    friction_partials_dfr_dfc_sliding_func.init(ffr_input, friction_partial_dfr_dfc2.as_vec_basic(), symbolic_cse, opt_level);
}


void symbolicEquations::generateFloorContactForce() {
    // We only consider x and y-displacement
    DenseMatrix z({x1s_z});  // arbitrary z element
    RCP<const Symbol> floor = symbol("floor");

    DenseMatrix v(2, 1);
    subtract_matrix(node, node_0, v);

    DenseMatrix v_hat(2, 1);
    convert_to_unit_vector(v, v_hat);
    DenseMatrix v_squared(2, 1);
    v.elementwise_mul_matrix(v, v_squared);
    RCP<const Basic> v_n = sqrt(add(v_squared.as_vec_basic()));
    RCP<const Basic> v_n_scaled = mul(mul(div(one, dt), K2), v_n);

    RCP<const Basic> gamma = sub(div(integer(2), add(one, exp(mul(integer(-1), v_n_scaled)))), one);
    RCP<const Basic> pos_fn = sqrt(mul(fn, fn));
    RCP<const Basic> ffr_scalar = mul(mul(gamma, mu), pos_fn);
    DenseMatrix ffr(2, 1);

    v_hat.mul_scalar(ffr_scalar, ffr);

    DenseMatrix fn_vec({fn});

    vec_basic ffr_input {x1s_x, x1s_y, x1s_x0, x1s_y0, fn, mu, dt, K2};

    DenseMatrix floor_friction_partial_dfr_dx(2, 2);
    DenseMatrix floor_friction_partial_dfr_dfn(2, 1);
    jacobian(ffr, node, floor_friction_partial_dfr_dx);
    jacobian(ffr, fn_vec, floor_friction_partial_dfr_dfn);

    floor_friction_partials_dfr_dx_func.init(ffr_input, floor_friction_partial_dfr_dx.as_vec_basic(), symbolic_cse, opt_level);
    floor_friction_partials_dfr_dfn_func.init(ffr_input, floor_friction_partial_dfr_dfn.as_vec_basic(), symbolic_cse, opt_level);

    // here, we assume \gamma = 1
    ffr_scalar = mul(mu, pos_fn);

    v_hat.mul_scalar(ffr_scalar, ffr);

    DenseMatrix floor_friction_g1_partial_dfr_dx(2, 2);
    DenseMatrix floor_friction_g1_partial_dfr_dfn(2, 1);
    jacobian(ffr, node, floor_friction_g1_partial_dfr_dx);
    jacobian(ffr, fn_vec, floor_friction_g1_partial_dfr_dfn);

    floor_friction_partials_gamma1_dfr_dx_func.init(ffr_input, floor_friction_g1_partial_dfr_dx.as_vec_basic(), symbolic_cse, opt_level);
    floor_friction_partials_gamma1_dfr_dfn_func.init(ffr_input, floor_friction_g1_partial_dfr_dfn.as_vec_basic(), symbolic_cse, opt_level);
}


void symbolicEquations::generateFloorFrictionJacobianFunctions() {
    // We only consider x and y-displacement
    DenseMatrix node({x1s_x, x1s_y});
    DenseMatrix node_0({x1s_x0, x1s_y0});
    RCP<const Symbol> fn = symbol("fn");

    DenseMatrix v(2, 1);
    subtract_matrix(node, node_0, v);

    DenseMatrix v_hat(2, 1);
    convert_to_unit_vector(v, v_hat);
    DenseMatrix v_squared(2, 1);
    v.elementwise_mul_matrix(v, v_squared);
    RCP<const Basic> v_n = sqrt(add(v_squared.as_vec_basic()));
    RCP<const Basic> v_n_scaled = mul(mul(div(one, dt), K2), v_n);

    RCP<const Basic> gamma = sub(div(integer(2), add(one, exp(mul(integer(-1), v_n_scaled)))), one);
    RCP<const Basic> pos_fn = sqrt(mul(fn, fn));
    RCP<const Basic> ffr_scalar = mul(mul(gamma, mu), pos_fn);
    DenseMatrix ffr(2, 1);

    v_hat.mul_scalar(ffr_scalar, ffr);

    DenseMatrix fn_vec({fn});

    vec_basic ffr_input {x1s_x, x1s_y, x1s_x0, x1s_y0, fn, mu, dt, K2};

    DenseMatrix floor_friction_partial_dfr_dx(2, 2);
    DenseMatrix floor_friction_partial_dfr_dfn(2, 1);
    jacobian(ffr, node, floor_friction_partial_dfr_dx);
    jacobian(ffr, fn_vec, floor_friction_partial_dfr_dfn);

    floor_friction_partials_dfr_dx_func.init(ffr_input, floor_friction_partial_dfr_dx.as_vec_basic(), symbolic_cse, opt_level);
    floor_friction_partials_dfr_dfn_func.init(ffr_input, floor_friction_partial_dfr_dfn.as_vec_basic(), symbolic_cse, opt_level);

    // here, we assume \gamma = 1
    ffr_scalar = mul(mu, pos_fn);

    v_hat.mul_scalar(ffr_scalar, ffr);

    DenseMatrix floor_friction_g1_partial_dfr_dx(2, 2);
    DenseMatrix floor_friction_g1_partial_dfr_dfn(2, 1);
    jacobian(ffr, node, floor_friction_g1_partial_dfr_dx);
    jacobian(ffr, fn_vec, floor_friction_g1_partial_dfr_dfn);

    floor_friction_partials_gamma1_dfr_dx_func.init(ffr_input, floor_friction_g1_partial_dfr_dx.as_vec_basic(), symbolic_cse, opt_level);
    floor_friction_partials_gamma1_dfr_dfn_func.init(ffr_input, floor_friction_g1_partial_dfr_dfn.as_vec_basic(), symbolic_cse, opt_level);
}