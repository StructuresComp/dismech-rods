#include "collision_detector.h"
#include "rod_mechanics/elastic_joint.h"
#include "rod_mechanics/elastic_rod.h"
#include "rod_mechanics/soft_robots.h"
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/broadphase/default_broadphase_callbacks.h>
#include <fcl/narrowphase/collision.h>

/*
 * We'll do collision detection using floats rather than doubles since it is
 * broadphase collision detection + for efficiency purposes.
 * The actual distance computation will be done using doubles later.
 */

CollisionDetector::CollisionDetector(const std::shared_ptr<SoftRobots>& soft_robots,
                                     double col_limit, double delta, bool self_contact)
    : soft_robots(soft_robots), delta(delta), col_limit(col_limit), self_contact(self_contact),
      a(0.0, 0.0, 1.0), num_collisions(0) {
    int index = 0;
    // First setup the limb edge ids. This has to be done
    // first otherwise memory locations may get rewritten
    for (const auto& limb : soft_robots->limbs) {
        limb_edge_ids.emplace_back();
        for (size_t i = 0; i < limb->ne; i++) {
            limb_edge_ids[index].emplace_back(index, i);
        }
        index++;
    }

    index = 0;
    int num_edges = 0;
    for (const auto& limb : soft_robots->limbs) {
        collision_managers.push_back(new fcl::DynamicAABBTreeCollisionManagerf());
        capsules.emplace_back();

        for (int i = 0; i < limb->ne; i++) {
            // Just init a random length. This will be updated later whenever we
            // do broadphase detection Also notice that we add a distance buffer
            // to the radius for proper IMC force computation
            std::shared_ptr<fcl::Capsulef> shape =
                std::make_shared<fcl::Capsulef>(limb->rod_radius + 0.5 * col_limit, 1.0);

            // For each capsule, set limb and node ids
            shape->setUserData(&(limb_edge_ids[index][i]));

            capsules[index].push_back(new fcl::CollisionObjectf(shape));
        }
        collision_managers[index]->registerObjects(capsules[index]);
        collision_managers[index]->setup();

        num_edges += limb->ne;
        index++;
    }

    // Try to make sure expensive std::vector reallocations never happen
    int conservative_guess = int(0.3 * pow(num_edges, 2));
    broad_phase_collision_set.reserve(conservative_guess);
    contact_ids.reserve(conservative_guess);
}

CollisionDetector::~CollisionDetector() {
    for (auto& manager : collision_managers) {
        delete manager;
    }
    for (auto& capsule_vector : capsules) {
        for (auto& capsule : capsule_vector) {
            delete capsule;
        }
    }
}

void CollisionDetector::prepCapsules() {
    int index = 0;
    Eigen::Vector3f dist_vec;
    Eigen::Vector3f center;
    fcl::CollisionObjectf* edge;
    for (const auto& limb : soft_robots->limbs) {
        for (int i = 0; i < limb->ne; i++) {
            edge = capsules[index][i];

            dist_vec = (limb->x.segment(4 * (i + 1), 3) - limb->x.segment(4 * i, 3)).cast<float>();
            float dist = dist_vec.norm();

            // Cast to derived class, so we can change capsule length (from
            // stretching)
            std::shared_ptr<fcl::Capsulef> x =
                (const std::shared_ptr<fcl::Capsule<float>>&)edge->collisionGeometry();
            x->lz = dist;

            // Set position of edge
            center = (limb->x.segment(4 * i, 3)).cast<float>() + 0.5 * dist_vec;
            edge->setTranslation(center);

            // Set orientation of edge
            dist_vec /= dist;
            getRotMat(dist_vec);
            edge->setRotation(rot_mat);
        }
        index++;
    }
}

void CollisionDetector::getRotMat(Eigen::Vector3f& b) {
    rot_mat.setIdentity();
    if (a == b) {
        return;
    }

    if (a == -b) {
        rot_mat(1, 1) = -1;
        rot_mat(2, 2) = -1;
        return;
    }

    Eigen::Vector3f v = a.cross(b);
    double c = a.dot(b);

    Eigen::Matrix3f kmat;
    kmat << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;

    rot_mat += kmat + kmat * kmat * (1.0 / (1.0 + c));
}

void CollisionDetector::broadPhaseCollisionDetection() {
    num_collisions = 0;
    broad_phase_collision_set.clear();
    prepCapsules();

    for (const auto& cm : collision_managers) {
        cm->update();
    }

    fcl::DefaultCollisionData<float> collision_data;
    collision_data.request.num_max_contacts = 1e10;  // arbitrarily large number
    for (size_t i = 0; i < soft_robots->limbs.size(); i++) {
        auto m1 = collision_managers[i];
        for (size_t j = i + 1; j < soft_robots->limbs.size(); j++) {
            auto m2 = collision_managers[j];

            // Check collisions between different limbs
            m1->collide(m2, &collision_data, fcl::DefaultCollisionFunction);

            std::vector<fcl::Contactf> contacts;
            collision_data.result.getContacts(contacts);

            for (const auto& contact : contacts) {
                broad_phase_collision_set.emplace_back((LimbEdgeInfo*)contact.o1->getUserData(),
                                                       (LimbEdgeInfo*)contact.o2->getUserData());
            }
            num_collisions += collision_data.result.numContacts();
        }
    }

    if (!self_contact)
        return;

    for (size_t i = 0; i < soft_robots->limbs.size(); i++) {
        auto m = collision_managers[i];

        m->collide(&collision_data, fcl::DefaultCollisionFunction);

        std::vector<fcl::Contactf> contacts;
        collision_data.result.getContacts(contacts);

        for (const auto& contact : contacts) {
            auto d1 = (LimbEdgeInfo*)contact.o1->getUserData();
            auto d2 = (LimbEdgeInfo*)contact.o2->getUserData();

            // Ignore edges that are within 4 ids within eachother
            // TODO: don't hardcode this, think of better way
            if (abs(d1->edge_id - d2->edge_id) < 3) {
                continue;
            }

            broad_phase_collision_set.emplace_back(d1, d2);
            num_collisions++;
        }
    }
}

bool CollisionDetector::fixBound(double& x) {
    if (x > 1) {
        x = 1;
        return true;
    }
    else if (x < 0) {
        x = 0;
        return true;
    }
    return false;
}

void CollisionDetector::lumelskyMinDist(int& idx1, int& idx2, int& idx3, int& idx4, int& idx5,
                                        int& idx6, double& dist, ConstraintType& constraint_type) {

    Vec3 x1s = soft_robots->limbs[idx5]->getVertex(idx1);
    Vec3 x1e = soft_robots->limbs[idx5]->getVertex(idx1 + 1);

    Vec3 x2s = soft_robots->limbs[idx6]->getVertex(idx2);
    Vec3 x2e = soft_robots->limbs[idx6]->getVertex(idx2 + 1);

    Vec3 e1 = x1e - x1s;
    Vec3 e2 = x2e - x2s;
    Vec3 e12 = x2s - x1s;

    double D1 = e1.dot(e1);
    double D2 = e2.dot(e2);
    double S1 = e1.dot(e12);
    double S2 = e2.dot(e12);
    double R = e1.dot(e2);

    double den = D1 * D2 - pow(R, 2);

    double t = 0.0;
    if (den != 0) {
        t = (S1 * D2 - S2 * R) / den;
    }
    fixBound(t);

    double u = (t * R - S2) / D2;

    double uf = u;

    if (fixBound(uf)) {
        t = (uf * R + S1) / D1;
    }
    fixBound(t);

    // Constraint type classification
    if ((t == 0 || t == 1) && (uf == 0 || uf == 1))  // p2p
    {
        if (t == 0) {
            idx3 = idx1 + 1;
        }
        else if (t == 1) {
            idx3 = idx1;
            idx1 = idx1 + 1;
        }
        if (uf == 0) {
            idx4 = idx2 + 1;
        }
        else if (uf == 1) {
            idx4 = idx2;
            idx2 = idx2 + 1;
        };
        constraint_type = POINT_TO_POINT;
    }
    else if (t == 0 || t == 1 || uf == 0 || uf == 1)  // p2e
    {
        if (t == 0) {
            int temp = idx1;
            idx1 = idx2;
            idx2 = temp;
            idx3 = idx1 + 1;
            idx4 = idx2 + 1;

            temp = idx5;
            idx5 = idx6;
            idx6 = temp;
        }
        else if (t == 1) {
            int temp = idx1 + 1;
            idx1 = idx2;
            idx2 = temp;
            idx3 = idx1 + 1;
            idx4 = idx2 - 1;

            temp = idx5;
            idx5 = idx6;
            idx6 = temp;
        }
        else if (uf == 0) {
            idx3 = idx1 + 1;
            idx4 = idx2 + 1;
        }
        else if (uf == 1) {
            idx2 = idx2 + 1;
            idx3 = idx1 + 1;
            idx4 = idx2 - 1;
        }
        constraint_type = POINT_TO_EDGE;
    }
    else {
        idx3 = idx1 + 1;
        idx4 = idx2 + 1;
        constraint_type = EDGE_TO_EDGE;
    }
    dist = (e1 * t - e2 * uf - e12).norm();
}

void CollisionDetector::narrowPhaseCollisionDetection() {
    num_collisions = 0;
    contact_ids.clear();
    // first four are nodes, last two are limb ids
    int idx1, idx2, idx3, idx4, idx5, idx6;
    ConstraintType constraint_type;
    double curr_dist;

    if (!broad_phase_collision_set.empty())
        min_dist = 1e10;

    for (const auto& possible_contact : broad_phase_collision_set) {
        // idx3 and idx4 will be populated by lumelskyAlg function call
        idx1 = possible_contact.c1->edge_id;
        idx2 = possible_contact.c2->edge_id;
        idx5 = possible_contact.c1->limb_id;
        idx6 = possible_contact.c2->limb_id;

        lumelskyMinDist(idx1, idx2, idx3, idx4, idx5, idx6, curr_dist, constraint_type);

        // Maybe we'll store this later
        double centerline_to_centerline_dist =
            soft_robots->limbs[idx5]->rod_radius + soft_robots->limbs[idx6]->rod_radius;
        double contact_limit = centerline_to_centerline_dist + delta;
        double numerical_limit = centerline_to_centerline_dist - delta;

        if (curr_dist - centerline_to_centerline_dist < min_dist) {
            min_dist = curr_dist - centerline_to_centerline_dist;
        }

        if (curr_dist < contact_limit) {
            contact_ids.emplace_back(idx1, idx2, idx3, idx4, idx5, idx6, constraint_type,
                                     (curr_dist > numerical_limit) ? NON_PENETRATED : PENETRATED);
            num_collisions++;
        }
    }
}
