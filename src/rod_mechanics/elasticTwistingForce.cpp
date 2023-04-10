#include "elasticTwistingForce.h"
#include "../time_steppers/baseTimeStepper.h"

elasticTwistingForce::elasticTwistingForce(const vector<shared_ptr<elasticRod>>& m_limbs,
                                           const vector<shared_ptr<elasticJoint>>& m_joints) :
                                           baseForce(m_limbs, m_joints)
{
    for (const auto& limb : m_limbs) {
        gradTwists.push_back(make_shared<MatrixXd>(MatrixXd::Zero(limb->nv,11)));
        deltams.push_back(make_shared<VectorXd>(VectorXd::Zero(limb->ne)));
        theta_fs.push_back(make_shared<VectorXd>(VectorXd::Zero(limb->ne)));
        theta_es.push_back(make_shared<VectorXd>(VectorXd::Zero(limb->ne)));
    }

    for (const auto& joint : m_joints) {
        int nb = joint->num_bending_combos;
        gradTwists.push_back(make_shared<MatrixXd>(MatrixXd::Zero(nb,11)));
        deltams.push_back(make_shared<VectorXd>(VectorXd::Zero(nb)));
        theta_fs.push_back(make_shared<VectorXd>(VectorXd::Zero(nb)));
        theta_es.push_back(make_shared<VectorXd>(VectorXd::Zero(nb)));
    }

    DDtwist.setZero(11,11);
    Jtt.setZero(11,11);
    gradTwistLocal.setZero(11);
    f.setZero(11);
}

elasticTwistingForce::~elasticTwistingForce()
{
    ;
}

void elasticTwistingForce::computeFt()
{
    int limb_idx = 0;
    for (const auto& limb : limbs) {
        GJ = limb->GJ;
        gradTwist = gradTwists[limb_idx];
        deltam = deltams[limb_idx];
        theta_f = theta_fs[limb_idx];
        theta_e = theta_es[limb_idx];

        for (int i = 0; i < limb->ne; i++)
        {
            theta_f->coeffRef(i) = limb->x(4*i+3);
        }

        for (int i = 0; i < limb->ne; i++)
        {
            if (i==0)
                theta_e->coeffRef(i)=0;
            else
                theta_e->coeffRef(i)=theta_f->coeff(i-1);
        }

        *deltam = *theta_f - *theta_e;

        for(int i = 1; i < limb->ne; i++)
        {
            norm_e = limb->edgeLen(i-1);
            norm_f = limb->edgeLen(i);
            gradTwist->row(i).segment(0,3) = -0.5 / norm_e * (limb->kb).row(i);
            gradTwist->row(i).segment(8,3) = 0.5 / norm_f * (limb->kb).row(i);
            gradTwist->row(i).segment(4,3) = -(gradTwist->row(i).segment(0,3)+gradTwist->row(i).segment(8,3));
            gradTwist->coeffRef(i, 3) = -1;
            gradTwist->coeffRef(i, 7) =  1;
        }

        for(int i = 1; i < limb->ne; i++)
        {
            value = GJ / limb->voronoiLen(i) * (deltam->coeff(i) + limb->refTwist (i) - limb->twistBar(i));
            f = -value * gradTwist->row(i);

            if (limb->isNodeJoint[i-1] != 1 && limb->isNodeJoint[i] != 1 && limb->isNodeJoint[i+1] != 1) {
                ci = 4*i-4;
                for (int k = 0; k < 11; k++) {
                    ind = ci + k;
                    stepper->addForce(ind, -f[k], limb_idx); // subtracting elastic force
                }
            }
            else {
                int n1, n2, n3, l1, l2, l3;
                n1 = limb->joint_ids[i-1].first;
                l1 = limb->joint_ids[i-1].second;
                n2 = limb->joint_ids[i].first;
                l2 = limb->joint_ids[i].second;
                n3 = limb->joint_ids[i+1].first;
                l3 = limb->joint_ids[i+1].second;

                // Nodal forces
                for (int k = 0; k < 3; k++) {
                    stepper->addForce(4*n1+k, -f[k], l1);
                    stepper->addForce(4*n2+k, -f[k+4], l2);
                    stepper->addForce(4*n3+k, -f[k+8], l3);
                }
                ci = 4*i-4;
                stepper->addForce(ci+3, -f[3], limb_idx);
                stepper->addForce(ci+7, -f[7], limb_idx);
            }

        }
        limb_idx++;
    }

    int joint_idx = 0;
    int theta1_i;
    int theta2_i;
    int n1, l1;
    int n2, l2;
    int n3, l3;
    int curr_iter = 0;
    int sgn1, sgn2;
    for (const auto& joint : joints) {
        GJ = limbs[0]->GJ;  // NOTE: CHANGE THIS LATER
        gradTwist = gradTwists[limbs.size()+joint_idx];
        deltam = deltams[limbs.size()+joint_idx];
        theta_f = theta_fs[limbs.size()+joint_idx];
        theta_e = theta_es[limbs.size()+joint_idx];

        curr_iter = 0;
        for (int i = 0; i < joint->ne; i++)  {
            l1 = joint->connected_nodes[i].second;
            for (int j = i+1; j < joint->ne; j++) {
                l3 = joint->connected_nodes[j].second;

                sgn1 = joint->sgns[curr_iter][0];
                sgn2 = joint->sgns[curr_iter][1];
                theta1_i = joint->theta_inds[curr_iter][0];
                theta2_i = joint->theta_inds[curr_iter][1];

                theta_e->coeffRef(curr_iter) = limbs[l1]->x(theta1_i) * sgn1;
                theta_f->coeffRef(curr_iter) = limbs[l3]->x(theta2_i) * sgn2;

                curr_iter++;
            }
        }

        *deltam = *theta_f - *theta_e;

        curr_iter = 0;
        for (int i = 0; i < joint->ne; i++) {
            for (int j = i+1; j < joint->ne; j++) {
                norm_e = joint->edge_len(i);
                norm_f = joint->edge_len(j);
                gradTwist->row(curr_iter).segment(0,3) = -0.5 / norm_e * (joint->kb).row(curr_iter);
                gradTwist->row(curr_iter).segment(8,3) = 0.5 / norm_f * (joint->kb).row(curr_iter);
                gradTwist->row(curr_iter).segment(4,3) = -(gradTwist->row(curr_iter).segment(0,3) +
                                                                     gradTwist->row(curr_iter).segment(8,3));
                gradTwist->coeffRef(curr_iter, 3) = -1;
                gradTwist->coeffRef(curr_iter, 7) =  1;
                curr_iter++;
            }
        }

        curr_iter = 0;
        n2 = joint->joint_node;
        l2 = joint->joint_limb;
        for (int i = 0; i < joint->ne; i++) {
            n1 = joint->connected_nodes[i].first;
            l1 = joint->connected_nodes[i].second;
            for (int j = i+1; j < joint->ne; j++) {
                n3 = joint->connected_nodes[j].first;
                l3 = joint->connected_nodes[j].second;

                sgn1 = joint->sgns[curr_iter][0];
                sgn2 = joint->sgns[curr_iter][1];
                theta1_i = joint->theta_inds[curr_iter][0];
                theta2_i = joint->theta_inds[curr_iter][1];

                value = GJ / joint->voronoi_len(curr_iter) * (deltam->coeff(curr_iter) +
                        joint->ref_twist(curr_iter) - joint->twistBar(curr_iter));
                f = -value * gradTwist->row(curr_iter);

                // Nodal forces
                for (int k = 0; k < 3; k++) {
                    stepper->addForce(4*n1+k,-f[k], l1);
                    stepper->addForce(4*n2+k,-f[k+4], l2);
                    stepper->addForce(4*n3+k,-f[k+8], l3);
                }
                // Theta moments
                stepper->addForce(theta1_i, -f[3] * sgn1, l1);
                stepper->addForce(theta2_i, -f[7] * sgn2, l3);

                curr_iter++;
            }
        }
        joint_idx++;
    }

}

void elasticTwistingForce::computeJt()
{
    int limb_idx = 0;
    for (const auto& limb : limbs) {
        GJ = limb->GJ;
        gradTwist = gradTwists[limb_idx];
        deltam = deltams[limb_idx];
        for (int i = 1; i < limb->ne; i++) {
            norm_e = limb->edgeLen(i - 1);
            norm_f = limb->edgeLen(i);
            te = limb->tangent.row(i - 1);
            tf = limb->tangent.row(i);

            norm2_e = norm_e * norm_e;
            norm2_f = norm_f * norm_f;

            kbLocal = (limb->kb).row(i);

            chi = 1.0 + te.dot(tf);
            tilde_t = (te + tf) / chi;

            crossMat(te, teMatrix);

            D2mDe2 = -0.25 / norm2_e * (kbLocal * (te + tilde_t).transpose()
                                        + (te + tilde_t) * kbLocal.transpose());
            D2mDf2 = -0.25 / norm2_f * (kbLocal * (tf + tilde_t).transpose()
                                        + (tf + tilde_t) * kbLocal.transpose());
            D2mDeDf = 0.5 / (norm_e * norm_f) * (2.0 / chi * teMatrix
                                                 - kbLocal * tilde_t.transpose());
            D2mDfDe = D2mDeDf.transpose();

            DDtwist.block(0, 0, 3, 3) = D2mDe2;
            DDtwist.block(0, 4, 3, 3) = -D2mDe2 + D2mDeDf;
            DDtwist.block(4, 0, 3, 3) = -D2mDe2 + D2mDfDe;
            DDtwist.block(4, 4, 3, 3) = D2mDe2 - (D2mDeDf + D2mDfDe) + D2mDf2;
            DDtwist.block(0, 8, 3, 3) = -D2mDeDf;
            DDtwist.block(8, 0, 3, 3) = -D2mDfDe;
            DDtwist.block(8, 4, 3, 3) = D2mDfDe - D2mDf2;
            DDtwist.block(4, 8, 3, 3) = D2mDeDf - D2mDf2;
            DDtwist.block(8, 8, 3, 3) = D2mDf2;

            gradTwistLocal = gradTwist->row(i);

            milen = -1 / limb->voronoiLen(i);

            // TODO: check that deltam is being properly saved from computeFt
            Jtt = GJ * milen * ((deltam->coeff(i) + limb->refTwist(i) - limb->twistBar(i))
                                * DDtwist + gradTwistLocal * gradTwistLocal.transpose());


            if (limb->isNodeJoint[i-1] != 1 && limb->isNodeJoint[i] != 1 && limb->isNodeJoint[i+1] != 1) {
                for (int j = 0; j < 11; j++) {
                    for (int k = 0; k < 11; k++) {
                        ind1 = 4 * i - 4 + j;
                        ind2 = 4 * i - 4 + k;
                        stepper->addJacobian(ind1, ind2, -Jtt(k, j), limb_idx);

                    }
                }
            }
            else {
                int n1, n2, n3, l1, l2, l3;
                n1 = limb->joint_ids[i-1].first;
                l1 = limb->joint_ids[i-1].second;
                n2 = limb->joint_ids[i].first;
                l2 = limb->joint_ids[i].second;
                n3 = limb->joint_ids[i+1].first;
                l3 = limb->joint_ids[i+1].second;

                for (int t = 0; t < 3; t++) {
                    for (int k = 0; k < 3; k++) {
                        stepper->addJacobian(4*n1+t, 4*n1+k, -Jtt(k, t), l1);
                        stepper->addJacobian(4*n1+t, 4*n2+k, -Jtt(k+4, t), l1, l2);
                        stepper->addJacobian(4*n1+t, 4*n3+k, -Jtt(k+8, t), l1, l3);

                        stepper->addJacobian(4*n2+t, 4*n1+k, -Jtt(k, t+4), l2, l1);
                        stepper->addJacobian(4*n2+t, 4*n2+k, -Jtt(k+4, t+4), l2);
                        stepper->addJacobian(4*n2+t, 4*n3+k, -Jtt(k+8, t+4), l2, l3);

                        stepper->addJacobian(4*n3+t, 4*n1+k, -Jtt(k, t+8), l3, l1);
                        stepper->addJacobian(4*n3+t, 4*n2+k, -Jtt(k+4, t+8), l3, l2);
                        stepper->addJacobian(4*n3+t, 4*n3+k, -Jtt(k+8, t+8), l3);
                    }
                }

                ci = 4 * (i-1);
                int n1_i = 4 * n1;
                int n2_i = 4 * n2;
                int n3_i = 4 * n3;
                for (int k = 0; k < 3; k++) {
                    stepper->addJacobian(ci+3, n1_i+k, -Jtt(k, 3), limb_idx, l1);
                    stepper->addJacobian(n1_i+k, ci+3, -Jtt(3, k), l1, limb_idx);
                    stepper->addJacobian(ci+3, n2_i+k, -Jtt(k+4, 3), limb_idx, l2);
                    stepper->addJacobian(n2_i+k, ci+3, -Jtt(3, k+4), l2, limb_idx);
                    stepper->addJacobian(ci+3, n3_i+k, -Jtt(k+8, 3), limb_idx, l3);
                    stepper->addJacobian(n3_i+k, ci+3, -Jtt(3, k+8), l3, limb_idx);
                    stepper->addJacobian(ci+7, n1_i+k, -Jtt(k, 7), limb_idx, l1);
                    stepper->addJacobian(n1_i+k, ci+7, -Jtt(7, k), l1, limb_idx);
                    stepper->addJacobian(ci+7, n2_i+k, -Jtt(k+4, 7), limb_idx, l2);
                    stepper->addJacobian(n2_i+k, ci+7, -Jtt(7, k+4), l2, limb_idx);
                    stepper->addJacobian(ci+7, n3_i+k, -Jtt(k+8, 3), limb_idx, l3);
                    stepper->addJacobian(n3_i+k, ci+7, -Jtt(7, k+8), l3, limb_idx);
                }
                stepper->addJacobian(ci+3, ci+3, -Jtt(3, 3), limb_idx);
                stepper->addJacobian(ci+3, ci+7, -Jtt(7, 3), limb_idx);
                stepper->addJacobian(ci+7, ci+3, -Jtt(3, 7), limb_idx);
                stepper->addJacobian(ci+7, ci+7, -Jtt(7, 7), limb_idx);
            }

        }
        limb_idx++;
    }

    int joint_idx = 0;
    int sgn1, sgn2;
    int theta1_i;
    int theta2_i;
    int n1, l1;
    int n2, l2;
    int n3, l3;
    int curr_iter = 0;
    for (const auto& joint : joints) {
        curr_iter = 0;
        GJ = limbs[0]->GJ;  // NOTE: CHANGE THIS LATER
        gradTwist = gradTwists[limbs.size() + joint_idx];
        deltam = deltams[limbs.size()+joint_idx];
        n2 = joint->joint_node;
        l2 = joint->joint_limb;
        for (int i = 0; i < joint->ne; i++) {
            n1 = joint->connected_nodes[i].first;
            l1 = joint->connected_nodes[i].second;
            for (int j = i+1; j < joint->ne; j++) {
                n3 = joint->connected_nodes[j].first;
                l3 = joint->connected_nodes[j].second;

                sgn1 = joint->sgns[curr_iter][0];
                sgn2 = joint->sgns[curr_iter][1];
                theta1_i = joint->theta_inds[curr_iter][0];
                theta2_i = joint->theta_inds[curr_iter][1];

                norm_e = joint->edge_len(i);
                norm_f = joint->edge_len(j);
                te = joint->tangents.row(i) * sgn1;
                tf = joint->tangents.row(j) * sgn2;

                norm2_e = norm_e * norm_e;
                norm2_f = norm_f * norm_f;

                kbLocal = (joint->kb).row(curr_iter);

                chi = 1.0 + te.dot(tf);
                tilde_t = (te + tf) / chi;

                crossMat(te, teMatrix);

                D2mDe2 = -0.25 / norm2_e * (kbLocal * (te + tilde_t).transpose()
                                            + (te + tilde_t) * kbLocal.transpose());
                D2mDf2 = -0.25 / norm2_f * (kbLocal * (tf + tilde_t).transpose()
                                            + (tf + tilde_t) * kbLocal.transpose());
                D2mDeDf = 0.5 / (norm_e * norm_f) * (2.0 / chi * teMatrix
                                                     - kbLocal * tilde_t.transpose());
                D2mDfDe = D2mDeDf.transpose();

                DDtwist.block(0, 0, 3, 3) = D2mDe2;
                DDtwist.block(0, 4, 3, 3) = -D2mDe2 + D2mDeDf;
                DDtwist.block(4, 0, 3, 3) = -D2mDe2 + D2mDfDe;
                DDtwist.block(4, 4, 3, 3) = D2mDe2 - (D2mDeDf + D2mDfDe) + D2mDf2;
                DDtwist.block(0, 8, 3, 3) = -D2mDeDf;
                DDtwist.block(8, 0, 3, 3) = -D2mDfDe;
                DDtwist.block(8, 4, 3, 3) = D2mDfDe - D2mDf2;
                DDtwist.block(4, 8, 3, 3) = D2mDeDf - D2mDf2;
                DDtwist.block(8, 8, 3, 3) = D2mDf2;

                gradTwistLocal = gradTwist->row(curr_iter);

                milen = -1 / joint->voronoi_len(curr_iter);

                Jtt = GJ * milen * ((deltam->coeff(curr_iter) + joint->ref_twist(curr_iter) -
                                     joint->twistBar(curr_iter)) * DDtwist +
                                    gradTwistLocal * gradTwistLocal.transpose());

                if (sgn1 == -1) {
                    Jtt.col(3) = -Jtt.col(3);
                    Jtt.row(3) = -Jtt.row(3);
                }
                if (sgn2 == -1) {
                    Jtt.col(7) = -Jtt.col(7);
                    Jtt.row(7) = -Jtt.row(7);
                }

                // Nodal forces
                for (int t = 0; t < 3; t++) {
                    for (int k = 0; k < 3; k++) {
                        stepper->addJacobian(4 * n1 + t, 4 * n1 + k, -Jtt(k, t), l1);
                        stepper->addJacobian(4 * n1 + t, 4 * n2 + k, -Jtt(k + 4, t), l1, l2);
                        stepper->addJacobian(4 * n1 + t, 4 * n3 + k, -Jtt(k + 8, t), l1, l3);

                        stepper->addJacobian(4 * n2 + t, 4 * n1 + k, -Jtt(k, t + 4), l2, l1);
                        stepper->addJacobian(4 * n2 + t, 4 * n2 + k, -Jtt(k + 4, t + 4), l2);
                        stepper->addJacobian(4 * n2 + t, 4 * n3 + k, -Jtt(k + 8, t + 4), l2, l3);

                        stepper->addJacobian(4 * n3 + t, 4 * n1 + k, -Jtt(k, t + 8), l3, l1);
                        stepper->addJacobian(4 * n3 + t, 4 * n2 + k, -Jtt(k + 4, t + 8), l3, l2);
                        stepper->addJacobian(4 * n3 + t, 4 * n3 + k, -Jtt(k + 8, t + 8), l3);
                    }
                }

                for (int k = 0; k < 3; k++) {
                    // dtheta1/dx1 and dx1/dtheta1
                    stepper->addJacobian(theta1_i, 4*n1+k, -Jtt(k, 3), l1);
                    stepper->addJacobian(4*n1+k, theta1_i, -Jtt(3, k), l1);

                    // dtheta1/dx2 and dx2/dtheta1
                    stepper->addJacobian(theta1_i, 4*n2+k, -Jtt(k+4, 3), l1, l2);
                    stepper->addJacobian(4*n2+k, theta1_i, -Jtt(3, k+4), l2, l1);

                    // dtheta1/dx3 and dx3/dtheta1
                    stepper->addJacobian(theta1_i, 4*n3+k, -Jtt(k+8, 3), l1, l3);
                    stepper->addJacobian(4*n3+k, theta1_i, -Jtt(3, k+8), l3, l1);

                    // dtheta2/dx1 and dx1/dtheta2
                    stepper->addJacobian(theta2_i, 4*n1+k, -Jtt(k, 7), l3, l1);
                    stepper->addJacobian(4*n1+k, theta2_i, -Jtt(7, k), l1, l3);

                    // dtheta2/dx2 and dx2/dtheta2
                    stepper->addJacobian(theta2_i, 4*n2+k, -Jtt(k+4, 7), l3, l2);
                    stepper->addJacobian(4*n2+k, theta2_i, -Jtt(7, k+4), l2, l3);

                    // dtheta2/dx3 and dx3/dtheta2
                    stepper->addJacobian(theta2_i, 4*n3+k, -Jtt(k+8, 7), l3);
                    stepper->addJacobian(4*n3+k, theta2_i, -Jtt(7, k+8), l3);
                }
                // dtheta1/dtheta1
                stepper->addJacobian(theta1_i, theta1_i, -Jtt(3, 3), l1);
                // dtheta1/dtheta2
                stepper->addJacobian(theta1_i, theta2_i, -Jtt(7, 3), l1, l3);
                // dtheta2/dtheta1
                stepper->addJacobian(theta2_i, theta1_i, -Jtt(3, 7), l3, l1);
                // dtheta2/dtheta2
                stepper->addJacobian(theta2_i, theta2_i, -Jtt(7, 7), l3);

                curr_iter++;
            }
        }
        joint_idx++;
    }

}

// Utility
void elasticTwistingForce::crossMat(const Vector3d &a,Matrix3d &b)
{
    b << 0, -a(2), a(1),
         a(2), 0, -a(0),
         -a(1), a(0), 0;
}
