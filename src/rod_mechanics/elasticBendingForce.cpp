#include "elasticBendingForce.h"

elasticBendingForce::elasticBendingForce(vector<shared_ptr<elasticRod>> m_limbs,
                                         vector<shared_ptr<Joint>> m_joints, shared_ptr<timeStepper> m_stepper)
{
    limbs = m_limbs;
    joints = m_joints;
    stepper = m_stepper;

    Id3 << 1, 0, 0,
           0, 1, 0,
           0, 0, 1;

    for (const auto& limb : limbs) {
        double EI = limb->EI;
        Matrix2d EIMat;
        EIMat << EI, 0,
                 0, EI;
        EIMatrices.push_back(EIMat);

        int nv = limb->nv;
        gradKappa1s.push_back(make_shared<MatrixXd>(MatrixXd::Zero(nv, 11)));
        gradKappa2s.push_back(make_shared<MatrixXd>(MatrixXd::Zero(nv, 11)));
    }

    for (const auto& joint : joints) {
        int nb = joint->num_bending_combos;
        gradKappa1s.push_back(make_shared<MatrixXd>(MatrixXd::Zero(nb, 11)));
        gradKappa2s.push_back(make_shared<MatrixXd>(MatrixXd::Zero(nb, 11)));
    }

//    int nv = rod->nv;
//    gradKappa1 = MatrixXd::Zero(nv,11);
//    gradKappa2 = MatrixXd::Zero(nv,11);
    relevantPart = MatrixXd::Zero(11, 2);;
    DDkappa1 = MatrixXd::Zero(11,11);
    DDkappa2 = MatrixXd::Zero(11,11);
    Jbb = MatrixXd::Zero(11,11);

    D2kappa1De2.setZero(3,3);
    D2kappa1Df2.setZero(3,3);
    D2kappa1DeDf.setZero(3,3);
    D2kappa2De2.setZero(3,3);
    D2kappa2Df2.setZero(3,3);
    D2kappa2DeDf.setZero(3,3);
//    kappa11 = VectorXd::Zero(nv);
//    kappa22 = VectorXd::Zero(nv);
    f = VectorXd::Zero(11);
}

elasticBendingForce::~elasticBendingForce()
{
    ;
}

void elasticBendingForce::computeFb()
{
    int limb_idx = 0;
    for (const auto& limb : limbs) {
        gradKappa1 = gradKappa1s[limb_idx];
        gradKappa2 = gradKappa2s[limb_idx];
        for (int i = 1; i < limb->ne; i++)
        {
//            if (limb_idx == 0) continue;
            if (limb->isEdgeJoint[i] || limb->isEdgeJoint[i-1]) continue;
            norm_e = limb->edgeLen(i-1);
            norm_f = limb->edgeLen(i);
            te = limb->tangent.row(i-1);
            tf = limb->tangent.row(i);
            d1e = limb->m1.row(i-1);
            d2e = limb->m2.row(i-1);
            d1f = limb->m1.row(i);
            d2f = limb->m2.row(i);

            chi = 1.0 + te.dot(tf);
            tilde_t = (te+tf)/chi;
            tilde_d1 = (d1e+d1f)/chi;
            tilde_d2 = (d2e+d2f)/chi;

            kappa1 = limb->kappa(i,0);
            kappa2 = limb->kappa(i,1);

            Dkappa1De = (1.0/norm_e)*(-kappa1*tilde_t + tf.cross(tilde_d2));
            Dkappa1Df = (1.0/norm_f)*(-kappa1*tilde_t - te.cross(tilde_d2));
            Dkappa2De = (1.0/norm_e)*(-kappa2*tilde_t - tf.cross(tilde_d1));
            Dkappa2Df = (1.0/norm_f)*(-kappa2*tilde_t + te.cross(tilde_d1));

            gradKappa1->row(i).segment(0,3)=-Dkappa1De;
            gradKappa1->row(i).segment(4,3)= Dkappa1De - Dkappa1Df;
            gradKappa1->row(i).segment(8,3)= Dkappa1Df;

            gradKappa2->row(i).segment(0,3)=-Dkappa2De;
            gradKappa2->row(i).segment(4,3)= Dkappa2De - Dkappa2Df;
            gradKappa2->row(i).segment(8,3)= Dkappa2Df;

            kbLocal = (limb->kb).row(i);

            gradKappa1->coeffRef(i,3)=-0.5*kbLocal.dot(d1e);
            gradKappa1->coeffRef(i,7)=-0.5*kbLocal.dot(d1f);
            gradKappa2->coeffRef(i,3)=-0.5*kbLocal.dot(d2e);
            gradKappa2->coeffRef(i,7)=-0.5*kbLocal.dot(d2f);
        }

        // TODO: merge this into the for loop above
        for (int i = 1; i < limb->ne; i++)
        {
            if (limb->isEdgeJoint[i] || limb->isEdgeJoint[i-1]) continue;
            ci = 4*i-4;
            relevantPart.col(0) = gradKappa1->row(i);
            relevantPart.col(1) = gradKappa2->row(i);
            kappaL = (limb->kappa).row(i) - (limb->kappaBar).row(i);
            f = - relevantPart * EIMatrices[limb_idx] * kappaL / limb->voronoiLen(i);

//            cout << f.norm() << endl;

            for (int k = 0; k < 11; k++)
            {
                int ind = ci + k;
                stepper->addForce(ind, -f[k], limb_idx); // subtracting elastic force
            }
        }
        limb_idx++;
    }

//    // TODO: need to add bending energies between joint edges and limb edges!!!
//    int joint_idx = 0;
//    for (const auto& joint : joints) {
//        int curr_iter = 0;
//        gradKappa1 = gradKappa1s[limbs.size() + joint_idx];
//        gradKappa2 = gradKappa2s[limbs.size() + joint_idx];
//        for (int i = 0; i < joint->ne; i++) {
//            for (int j = i+1; j < joint->ne; j++) {
//                norm_e = joint->edge_len(i);
//                norm_f = joint->edge_len(j);
//                te = joint->tangents.row(i);
//                tf = -joint->tangents.row(j);
//                d1e = joint->m1.row(i);
//                d2e = joint->m2.row(i);
//                d1f = joint->m1.row(j);
//                d2f = joint->m2.row(j);
//
//                chi = 1.0 + te.dot(tf);
//                tilde_t = (te+tf)/chi;
//                tilde_d1 = (d1e+d1f)/chi;
//                tilde_d2 = (d2e+d2f)/chi;
//
//                kappa1 = joint->kappa(curr_iter, 0);
//                kappa2 = joint->kappa(curr_iter, 1);
//
//                Dkappa1De = (1.0/norm_e)*(-kappa1*tilde_t + tf.cross(tilde_d2));
//                Dkappa1Df = (1.0/norm_f)*(-kappa1*tilde_t - te.cross(tilde_d2));
//                Dkappa2De = (1.0/norm_e)*(-kappa2*tilde_t - tf.cross(tilde_d1));
//                Dkappa2Df = (1.0/norm_f)*(-kappa2*tilde_t + te.cross(tilde_d1));
//
//                gradKappa1->row(curr_iter).segment(0,3)=-Dkappa1De;
//                gradKappa1->row(curr_iter).segment(4,3)= Dkappa1De - Dkappa1Df;
//                gradKappa1->row(curr_iter).segment(8,3)= Dkappa1Df;
//
//                gradKappa2->row(curr_iter).segment(0,3)=-Dkappa2De;
//                gradKappa2->row(curr_iter).segment(4,3)= Dkappa2De - Dkappa2Df;
//                gradKappa2->row(curr_iter).segment(8,3)= Dkappa2Df;
//
//                kbLocal = (joint->kb).row(curr_iter);
//
//                gradKappa1->coeffRef(curr_iter,3)=-0.5*kbLocal.dot(d1e);
//                gradKappa1->coeffRef(curr_iter,7)=-0.5*kbLocal.dot(d1f);
//                gradKappa2->coeffRef(curr_iter,3)=-0.5*kbLocal.dot(d2e);
//                gradKappa2->coeffRef(curr_iter,7)=-0.5*kbLocal.dot(d2f);
//
//                curr_iter++;
//            }
//        }
//        curr_iter = 0;
//        int n1, l1;
//        int n2, l2;
//        int n3, l3;
//
//        n2 = joint->joint_node;
//        l2 = joint->joint_limb;
//        for (int i = 0; i < joint->ne; i++) {
//            n1 = joint->connected_nodes[i].first;
//            l1 = joint->connected_nodes[i].second;
//            for (int j = i + 1; j < joint->ne; j++) {
//                n3 = joint->connected_nodes[j].first;
//                l3 = joint->connected_nodes[j].second;
//                relevantPart.col(0) = gradKappa1->row(curr_iter);
//                relevantPart.col(1) = gradKappa2->row(curr_iter);
//                kappaL = joint->kappa.row(curr_iter) - joint->kappaBar.row(curr_iter);
//                // TODO: NEED TO CONSTRUCT UNIQUE EI MATRICES WITH EACH RODS EI
//                f = -relevantPart * EIMatrices[0] * kappaL / joint->voronoi_len(curr_iter);
//
//                for (int k = 0; k < 3; k++) {
//                    stepper->addForce(4*n1+k, -f[k], l1);
//                    stepper->addForce(4*n2+k, -f[k+4], l2);
//                    stepper->addForce(4*n3+k, -f[k+8], l3);
//                }
//
//                curr_iter++;
//            }
//        }
//        joint_idx++;
//    }
}

void elasticBendingForce::computeJb()
{
    int limb_idx = 0;
    for (const auto& limb : limbs) {
        gradKappa1 = gradKappa1s[limb_idx];
        gradKappa2 = gradKappa2s[limb_idx];
        for (int i = 1; i < limb->ne; i++) {
//            if (limb_idx == 0) continue;
            if (limb->isEdgeJoint[i] || limb->isEdgeJoint[i-1]) continue;
            norm_e = limb->edgeLen(i - 1);
            norm_f = limb->edgeLen(i);
            te = limb->tangent.row(i - 1);
            tf = limb->tangent.row(i);
            d1e = limb->m1.row(i - 1);
            d2e = limb->m2.row(i - 1);
            d1f = limb->m1.row(i);
            d2f = limb->m2.row(i);

            norm2_e = norm_e * norm_e;
            norm2_f = norm_f * norm_f;

            chi = 1.0 + te.dot(tf);
            tilde_t = (te + tf) / chi;
            tilde_d1 = (d1e + d1f) / chi;
            tilde_d2 = (d2e + d2f) / chi;

            kappa1 = limb->kappa(i, 0);
            kappa2 = limb->kappa(i, 1);

            kbLocal = (limb->kb).row(i);

            JacobianComputation();

            len = limb->voronoiLen(i);
            relevantPart.col(0) = gradKappa1->row(i);
            relevantPart.col(1) = gradKappa2->row(i);

            Jbb = -1.0 / len * relevantPart * EIMatrices[limb_idx] * relevantPart.transpose();

            kappaL = (limb->kappa).row(i) - (limb->kappaBar).row(i);

            temp = -1.0 / len * kappaL.transpose() * EIMatrices[limb_idx];

            Jbb = Jbb + temp(0) * DDkappa1 + temp(1) * DDkappa2;
//            cout << "Bending HERE" << endl;

//            if (limb_idx == 0) {
//                cout << Jbb(7, 7) << " " << Jbb(7, 9) << " " << Jbb(9, 7) << " " << Jbb(9, 9) << endl;
//            }

            for (int j = 0; j < 11; j++) {
                for (int k = 0; k < 11; k++) {
                    int ind1 = 4 * i - 4 + j;
                    int ind2 = 4 * i - 4 + k;

                    stepper->addJacobian(ind1, ind2, -Jbb(k, j), limb_idx);
                }
            }
        }
        limb_idx++;
    }

//    int joint_idx = 0;
//    for (const auto& joint : joints) {
//        int curr_iter = 0;
//        gradKappa1 = gradKappa1s[limbs.size() + joint_idx];
//        gradKappa2 = gradKappa2s[limbs.size() + joint_idx];
//        for (int i = 0; i < joint->ne; i++) {
//            for (int j = i+1; j < joint->ne; j++) {
//                norm_e = joint->edge_len(i);
//                norm_f = joint->edge_len(j);
//                te = joint->tangents.row(i);
//                tf = -joint->tangents.row(j);
//                d1e = joint->m1.row(i);
//                d2e = joint->m2.row(i);
//                d1f = joint->m1.row(j);
//                d2f = joint->m2.row(j);
//
//                norm2_e = norm_e * norm_e;
//                norm2_f = norm_f * norm_f;
//
//                chi = 1.0 + te.dot(tf);
//                tilde_t = (te + tf) / chi;
//                tilde_d1 = (d1e + d1f) / chi;
//                tilde_d2 = (d2e + d2f) / chi;
//
//                kappa1 = joint->kappa(curr_iter, 0);
//                kappa2 = joint->kappa(curr_iter, 1);
//
//                kbLocal = (joint->kb).row(curr_iter);
//
//                JacobianComputation();
//
//                len = joint->voronoi_len(curr_iter);
//                relevantPart.col(0) = gradKappa1->row(curr_iter);
//                relevantPart.col(1) = gradKappa2->row(curr_iter);
//
//                // TODO: ADD CORRECT EI MATRICES FOR JOINTS
//                Jbb = -1.0 / len * relevantPart * EIMatrices[0] * relevantPart.transpose();
//
//                kappaL = (joint->kappa).row(curr_iter) - (joint->kappaBar).row(curr_iter);
//
//                temp = -1.0 / len * kappaL.transpose() * EIMatrices[0];
//
//                Jbb = Jbb + temp(0) * DDkappa1 + temp(1) * DDkappa2;
//
//                int n1, l1;
//                int n2, l2;
//                int n3, l3;
//                n1 = joint->connected_nodes[i].first;
//                l1 = joint->connected_nodes[i].second;
//                n2 = joint->joint_node;
//                l2 = joint->joint_limb;
//                n3 = joint->connected_nodes[j].first;
//                l3 = joint->connected_nodes[j].second;
//
//                for (int t = 0; t < 3; t++) {
//                    for (int k = 0; k < 3; k++) {
//                        stepper->addJacobian(4*n1+t, 4*n1+k, -Jbb(k, t), l1);
//                        stepper->addJacobian(4*n1+t, 4*n2+k, -Jbb(k+4, t), l1, l2);
//                        stepper->addJacobian(4*n1+t, 4*n3+k, -Jbb(k+8, t), l1, l3);
//
//                        stepper->addJacobian(4*n2+t, 4*n1+k, -Jbb(k, t+4), l2, l1);
//                        stepper->addJacobian(4*n2+t, 4*n2+k, -Jbb(k+4, t+4), l2);
//                        stepper->addJacobian(4*n2+t, 4*n3+k, -Jbb(k+8, t+4), l2, l3);
//
//                        stepper->addJacobian(4*n3+t, 4*n1+k, -Jbb(k, t+8), l3, l1);
//                        stepper->addJacobian(4*n3+t, 4*n2+k, -Jbb(k+4, t+8), l3, l2);
//                        stepper->addJacobian(4*n3+t, 4*n3+k, -Jbb(k+8, t+8), l3);
//                    }
//                }
//
//                curr_iter++;
//            }
//        }
//
//        joint_idx++;
//    }

}

// Utility
void elasticBendingForce::crossMat(const Vector3d &a,Matrix3d &b)
{
    b << 0, -a(2), a(1),
         a(2), 0, -a(0),
         -a(1), a(0), 0;
}


void elasticBendingForce::JacobianComputation() {
    tt_o_tt = tilde_t * tilde_t.transpose();

    crossMat(tilde_d1, tilde_d1_3d);
    crossMat(tilde_d2, tilde_d2_3d);

    tmp = tf.cross(tilde_d2);
    tf_c_d2t_o_tt = tmp * tilde_t.transpose();
    tt_o_tf_c_d2t = tf_c_d2t_o_tt.transpose();
    kb_o_d2e = kbLocal * d2e.transpose();
    d2e_o_kb = kb_o_d2e.transpose();

    D2kappa1De2 =
            1.0 / norm2_e * (2 * kappa1 * tt_o_tt - tf_c_d2t_o_tt - tt_o_tf_c_d2t)
            - kappa1 / (chi * norm2_e) * (Id3 - te * te.transpose())
            + 1.0 / (4.0 * norm2_e) * (kb_o_d2e + d2e_o_kb);

    tmp = te.cross(tilde_d2);
    te_c_d2t_o_tt = tmp * tilde_t.transpose();
    tt_o_te_c_d2t = te_c_d2t_o_tt.transpose();
    kb_o_d2f = kbLocal * d2f.transpose();
    d2f_o_kb = kb_o_d2f.transpose();

    D2kappa1Df2 =
            1.0 / norm2_f * (2 * kappa1 * tt_o_tt + te_c_d2t_o_tt + tt_o_te_c_d2t)
            - kappa1 / (chi * norm2_f) * (Id3 - tf * tf.transpose())
            + 1.0 / (4.0 * norm2_f) * (kb_o_d2f + d2f_o_kb);

    D2kappa1DeDf =
            -kappa1 / (chi * norm_e * norm_f) * (Id3 + te * tf.transpose())
            + 1.0 / (norm_e * norm_f) * (2 * kappa1 * tt_o_tt - tf_c_d2t_o_tt +
                                         tt_o_te_c_d2t - tilde_d2_3d);
    D2kappa1DfDe = D2kappa1DeDf.transpose();

    tmp = tf.cross(tilde_d1);
    tf_c_d1t_o_tt = tmp * tilde_t.transpose();
    tt_o_tf_c_d1t = tf_c_d1t_o_tt.transpose();
    kb_o_d1e = kbLocal * d1e.transpose();
    d1e_o_kb = kb_o_d1e.transpose();

    D2kappa2De2
            = 1.0 / norm2_e * (2 * kappa2 * tt_o_tt + tf_c_d1t_o_tt + tt_o_tf_c_d1t)
              - kappa2 / (chi * norm2_e) * (Id3 - te * te.transpose())
              - 1.0 / (4.0 * norm2_e) * (kb_o_d1e + d1e_o_kb);

    tmp = te.cross(tilde_d1);
    te_c_d1t_o_tt = tmp * tilde_t.transpose();
    tt_o_te_c_d1t = te_c_d1t_o_tt.transpose();
    kb_o_d1f = kbLocal * d1f.transpose();
    d1f_o_kb = kb_o_d1f.transpose();

    D2kappa2Df2
            = 1.0 / norm2_f * (2 * kappa2 * tt_o_tt - te_c_d1t_o_tt - tt_o_te_c_d1t)
              - kappa2 / (chi * norm2_f) * (Id3 - tf * tf.transpose())
              - 1.0 / (4.0 * norm2_f) * (kb_o_d1f + d1f_o_kb);

    D2kappa2DeDf
            = -kappa2 / (chi * norm_e * norm_f) * (Id3 + te * tf.transpose())
              + 1.0 / (norm_e * norm_f) * (2 * kappa2 * tt_o_tt + tf_c_d1t_o_tt
                                           - tt_o_te_c_d1t + tilde_d1_3d);

    D2kappa2DfDe = D2kappa2DeDf.transpose();

    D2kappa1Dthetae2 = -0.5 * kbLocal.dot(d2e);
    D2kappa1Dthetaf2 = -0.5 * kbLocal.dot(d2f);
    D2kappa2Dthetae2 = 0.5 * kbLocal.dot(d1e);
    D2kappa2Dthetaf2 = 0.5 * kbLocal.dot(d1f);

    D2kappa1DeDthetae
            = 1.0 / norm_e * ((0.5 * kbLocal.dot(d1e)) * tilde_t
                              - 1.0 / chi * (tf.cross(d1e)));
    D2kappa1DeDthetaf
            = 1.0 / norm_e * ((0.5 * kbLocal.dot(d1f)) * tilde_t
                              - 1.0 / chi * (tf.cross(d1f)));
    D2kappa1DfDthetae
            = 1.0 / norm_f * ((0.5 * kbLocal.dot(d1e)) * tilde_t
                              + 1.0 / chi * (te.cross(d1e)));
    D2kappa1DfDthetaf
            = 1.0 / norm_f * ((0.5 * kbLocal.dot(d1f)) * tilde_t
                              + 1.0 / chi * (te.cross(d1f)));
    D2kappa2DeDthetae
            = 1.0 / norm_e * ((0.5 * kbLocal.dot(d2e)) * tilde_t
                              - 1.0 / chi * (tf.cross(d2e)));
    D2kappa2DeDthetaf
            = 1.0 / norm_e * ((0.5 * kbLocal.dot(d2f)) * tilde_t
                              - 1.0 / chi * (tf.cross(d2f)));
    D2kappa2DfDthetae
            = 1.0 / norm_f * ((0.5 * kbLocal.dot(d2e)) * tilde_t
                              + 1.0 / chi * (te.cross(d2e)));
    D2kappa2DfDthetaf
            = 1.0 / norm_f * ((0.5 * kbLocal.dot(d2f)) * tilde_t
                              + 1.0 / chi * (te.cross(d2f)));

    DDkappa1.block(0, 0, 3, 3) = D2kappa1De2;
    DDkappa1.block(0, 4, 3, 3) = -D2kappa1De2 + D2kappa1DeDf;
    DDkappa1.block(0, 8, 3, 3) = -D2kappa1DeDf;
    DDkappa1.block(4, 0, 3, 3) = -D2kappa1De2 + D2kappa1DfDe;
    DDkappa1.block(4, 4, 3, 3) = D2kappa1De2 - D2kappa1DeDf - D2kappa1DfDe + D2kappa1Df2;
    DDkappa1.block(4, 8, 3, 3) = D2kappa1DeDf - D2kappa1Df2;
    DDkappa1.block(8, 0, 3, 3) = -D2kappa1DfDe;
    DDkappa1.block(8, 4, 3, 3) = D2kappa1DfDe - D2kappa1Df2;
    DDkappa1.block(8, 8, 3, 3) = D2kappa1Df2;

    DDkappa1(3, 3) = D2kappa1Dthetae2;
    DDkappa1(7, 7) = D2kappa1Dthetaf2;

    DDkappa1.col(3).segment(0, 3) = -D2kappa1DeDthetae;
    DDkappa1.col(3).segment(4, 3) = D2kappa1DeDthetae - D2kappa1DfDthetae;
    DDkappa1.col(3).segment(8, 3) = D2kappa1DfDthetae;
    DDkappa1.row(3).segment(0, 3) = DDkappa1.col(3).segment(0, 3).transpose();
    DDkappa1.row(3).segment(4, 3) = DDkappa1.col(3).segment(4, 3).transpose();
    DDkappa1.row(3).segment(8, 3) = DDkappa1.col(3).segment(8, 3).transpose();

    DDkappa1.col(7).segment(0, 3) = -D2kappa1DeDthetaf;
    DDkappa1.col(7).segment(4, 3) = D2kappa1DeDthetaf - D2kappa1DfDthetaf;
    DDkappa1.col(7).segment(8, 3) = D2kappa1DfDthetaf;
    DDkappa1.row(7).segment(0, 3) = DDkappa1.col(7).segment(0, 3).transpose();
    DDkappa1.row(7).segment(4, 3) = DDkappa1.col(7).segment(4, 3).transpose();
    DDkappa1.row(7).segment(8, 3) = DDkappa1.col(7).segment(8, 3).transpose();

    DDkappa2.block(0, 0, 3, 3) = D2kappa2De2;
    DDkappa2.block(0, 4, 3, 3) = -D2kappa2De2 + D2kappa2DeDf;
    DDkappa2.block(0, 8, 3, 3) = -D2kappa2DeDf;
    DDkappa2.block(4, 0, 3, 3) = -D2kappa2De2 + D2kappa2DfDe;
    DDkappa2.block(4, 4, 3, 3) = D2kappa2De2 - D2kappa2DeDf - D2kappa2DfDe + D2kappa2Df2;
    DDkappa2.block(4, 8, 3, 3) = D2kappa2DeDf - D2kappa2Df2;
    DDkappa2.block(8, 0, 3, 3) = -D2kappa2DfDe;
    DDkappa2.block(8, 4, 3, 3) = D2kappa2DfDe - D2kappa2Df2;
    DDkappa2.block(8, 8, 3, 3) = D2kappa2Df2;

    DDkappa2(3, 3) = D2kappa2Dthetae2;
    DDkappa2(7, 7) = D2kappa2Dthetaf2;

    DDkappa2.col(3).segment(0, 3) = -D2kappa2DeDthetae;
    DDkappa2.col(3).segment(4, 3) = D2kappa2DeDthetae - D2kappa2DfDthetae;
    DDkappa2.col(3).segment(8, 3) = D2kappa2DfDthetae;
    DDkappa2.row(3).segment(0, 3) = DDkappa2.col(3).segment(0, 3).transpose();
    DDkappa2.row(3).segment(4, 3) = DDkappa2.col(3).segment(4, 3).transpose();
    DDkappa2.row(3).segment(8, 3) = DDkappa2.col(3).segment(8, 3).transpose();

    DDkappa2.col(7).segment(0, 3) = -D2kappa2DeDthetaf;
    DDkappa2.col(7).segment(4, 3) = D2kappa2DeDthetaf - D2kappa2DfDthetaf;
    DDkappa2.col(7).segment(8, 3) = D2kappa2DfDthetaf;
    DDkappa2.row(7).segment(0, 3) = DDkappa2.col(7).segment(0, 3).transpose();
    DDkappa2.row(7).segment(4, 3) = DDkappa2.col(7).segment(4, 3).transpose();
    DDkappa2.row(7).segment(8, 3) = DDkappa2.col(7).segment(8, 3).transpose();
}
