#include "elastic_stretching_force.h"
#include "time_steppers/base_time_stepper.h"

ElasticStretchingForce::ElasticStretchingForce(const shared_ptr<SoftRobots>& m_soft_robots)
    : BaseForce(m_soft_robots) {
    f.setZero(3);
    Jss.setZero(7, 7);
    Id3 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
}

ElasticStretchingForce::~ElasticStretchingForce() {
    ;
}

void ElasticStretchingForce::computeForce(double dt) {
    int limb_idx = 0;
    for (const auto& limb : soft_robots->limbs) {

        for (int i = 0; i < limb->ne; i++) {
            if (limb->isEdgeJoint[i])
                continue;
            epsX = limb->edge_len(i) / limb->ref_len(i) - 1.0;
            f = limb->EA * (limb->tangent).row(i) * epsX;
            for (int k = 0; k < 3; k++) {
                ind = 4 * i + k;
                stepper->addForce(ind, -f[k],
                                  limb_idx);  // subtracting elastic force

                ind = 4 * (i + 1) + k;
                stepper->addForce(ind, f[k], limb_idx);  // adding elastic force
            }
        }
        limb_idx++;
    }

    shared_ptr<ElasticRod> curr_limb;
    int n1;
    int sgn;
    for (const auto& joint : soft_robots->joints) {
        for (int i = 0; i < joint->ne; i++) {
            joint->bending_twist_signs[i] == 1 ? sgn = 1 : sgn = -1;
            n1 = joint->connected_nodes[i].first;
            limb_idx = joint->connected_nodes[i].second;
            curr_limb = soft_robots->limbs[limb_idx];
            epsX = joint->edge_len(i) / joint->ref_len(i) - 1.0;
            f = curr_limb->EA * joint->tangents.row(i) * sgn * epsX;
            for (int k = 0; k < 3; k++) {
                ind = 4 * n1 + k;

                stepper->addForce(ind, -f[k], limb_idx);

                ind = 4 * joint->joint_node + k;

                stepper->addForce(ind, f[k], joint->joint_limb);
            }
        }
    }
}

void ElasticStretchingForce::computeForceAndJacobian(double dt) {
    computeForce(dt);

    int limb_idx = 0;
    for (const auto& limb : soft_robots->limbs) {
        for (int i = 0; i < limb->ne; i++) {
            if (limb->isEdgeJoint[i])
                continue;
            len = limb->edge_len[i];
            refLength = limb->ref_len[i];

            dxx(0) = limb->x(4 * i + 4) - limb->x(4 * i + 0);
            dxx(1) = limb->x(4 * i + 5) - limb->x(4 * i + 1);
            dxx(2) = limb->x(4 * i + 6) - limb->x(4 * i + 2);

            u = dxx;
            v = u.transpose();
            M0 = limb->EA *
                 ((1 / refLength - 1 / len) * Id3 + (1 / len) * (u * v) / (u.norm() * u.norm()));

            Jss.block(0, 0, 3, 3) = -M0;
            Jss.block(4, 4, 3, 3) = -M0;
            Jss.block(4, 0, 3, 3) = M0;
            Jss.block(0, 4, 3, 3) = M0;

            for (int j = 0; j < 7; j++) {
                for (int k = 0; k < 7; k++) {
                    ind1 = 4 * i + j;
                    ind2 = 4 * i + k;
                    stepper->addJacobian(ind1, ind2, -Jss(k, j), limb_idx);
                }
            }
        }
        limb_idx++;
    }

    int n1;
    shared_ptr<ElasticRod> curr_limb;
    for (const auto& joint : soft_robots->joints) {
        for (int i = 0; i < joint->ne; i++) {
            n1 = joint->connected_nodes[i].first;
            limb_idx = joint->connected_nodes[i].second;
            curr_limb = soft_robots->limbs[limb_idx];

            len = joint->edge_len(i);
            refLength = joint->ref_len(i);

            dxx(0) = joint->x(0) - curr_limb->x(4 * n1);
            dxx(1) = joint->x(1) - curr_limb->x(4 * n1 + 1);
            dxx(2) = joint->x(2) - curr_limb->x(4 * n1 + 2);

            u = dxx;
            v = u.transpose();
            M0 = curr_limb->EA *
                 ((1 / refLength - 1 / len) * Id3 + (1 / len) * (u * v) / (u.norm() * u.norm()));

            Jss.block(0, 0, 3, 3) = -M0;
            Jss.block(4, 4, 3, 3) = -M0;
            Jss.block(4, 0, 3, 3) = M0;
            Jss.block(0, 4, 3, 3) = M0;

            // TODO: fix the n1 only reference, i think this is wrong
            int l1 = limb_idx;
            int l2 = joint->joint_limb;
            int n2 = joint->joint_node;
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    stepper->addJacobian(4 * n1 + j, 4 * n1 + k, -Jss(k, j), l1);
                    stepper->addJacobian(4 * n1 + j, 4 * n2 + k, -Jss(k + 4, j), l1, l2);
                    stepper->addJacobian(4 * n2 + j, 4 * n1 + k, -Jss(k, j + 4), l2, l1);
                    stepper->addJacobian(4 * n2 + j, 4 * n2 + k, -Jss(k + 4, j + 4), l2);
                }
            }
        }
    }
}
