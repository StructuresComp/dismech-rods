#include "collisionDetector.h"



/*
 * We'll do collision detection using floats rather than doubles since it is
 * broadphase collision detection + for efficiency purposes.
 * The actual distance computation will be done using doubles later.
 */



collisionDetector::collisionDetector(const shared_ptr<softRobots> &m_soft_robots, double m_col_limit, double m_delta) :
                                    soft_robots(m_soft_robots), delta(m_delta), col_limit(m_col_limit),
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
        cylinders.emplace_back();

        for (int i = 0; i < limb->ne; i++) {
            // Just init a random length. This will be updated later whenever we do broadphase detection
            // Also notice that we add a distance buffer to the radius for proper IMC force computation
            shared_ptr<fcl::Cylinderf> shape = make_shared<fcl::Cylinderf>(limb->rod_radius + 0.5 * col_limit, 1.0);

            // For each cylinder, set limb and node ids
            shape->setUserData(&(limb_edge_ids[index][i]));

            cylinders[index].push_back(new fcl::CollisionObjectf(shape));
        }
        collision_managers[index]->registerObjects(cylinders[index]);
        collision_managers[index]->setup();

        num_edges += limb->ne;
        index++;
    }

    // Try to make sure expensive vector reallocations never happen
    int conservative_guess = int(0.3 * pow(num_edges, 2));
    broad_phase_collision_set.reserve(conservative_guess);
    contact_ids.reserve(conservative_guess);
}


void collisionDetector::prepCylinders() {
    int index = 0;
    Eigen::Vector3f dist_vec;
    Eigen::Vector3f center;
    fcl::CollisionObjectf* edge;
    for (const auto& limb : soft_robots->limbs) {
        for (int i = 0; i < limb->ne; i++) {
            edge = cylinders[index][i];

            dist_vec = (limb->x.segment(4*(i+1), 3) - limb->x.segment(4*i, 3)).cast<float>();
            float dist = dist_vec.norm();

            // Cast to derived class, so we can change cylinder length (from stretching)
            shared_ptr<fcl::Cylinderf> x = (const shared_ptr<fcl::Cylinder<float>> &) edge->collisionGeometry();
            x->lz = dist;

            // Set position of edge
            center = (limb->x.segment(4*i, 3)).cast<float>()  + 0.5 * dist_vec;
            edge->setTranslation(center);

            // Set orientation of edge
            dist_vec /= dist;
            getRotMat(dist_vec);
            edge->setRotation(rot_mat);
        }
        index++;
    }
}


void collisionDetector::getRotMat(Eigen::Vector3f &b) {
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
    kmat << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;

    rot_mat += kmat + kmat * kmat * (1.0 / (1.0 + c));
}


void collisionDetector::broadPhaseCollisionDetection() {
    num_collisions = 0;
    broad_phase_collision_set.clear();
    prepCylinders();

    for (const auto& cm : collision_managers) {
        cm->update();
    }

    fcl::DefaultCollisionData<float> collision_data;
    collision_data.request.num_max_contacts = 1e10;  // arbitrarily large number
    for (size_t i = 0; i < soft_robots->limbs.size(); i++) {
        auto m1 = collision_managers[i];
        for (size_t j = i+1; j < soft_robots->limbs.size(); j++) {
            auto m2 = collision_managers[j];

            // Check collisions between different limbs
            m1->collide(m2, &collision_data, fcl::DefaultCollisionFunction);

            vector<fcl::Contactf> contacts;
            collision_data.result.getContacts(contacts);

            for (const auto& contact : contacts) {
                broad_phase_collision_set.emplace_back((limb_edge_info*)contact.o1->getUserData(),
                                                       (limb_edge_info*)contact.o2->getUserData());
            }
            num_collisions += collision_data.result.numContacts();
        }
    }
    // TODO: add option for self-contact
    for (size_t i = 0; i < soft_robots->limbs.size(); i++) {
        auto m = collision_managers[i];

        m->collide(&collision_data, fcl::DefaultCollisionFunction);

        vector<fcl::Contactf> contacts;
        collision_data.result.getContacts(contacts);

        for (const auto& contact : contacts) {
            auto d1 = (limb_edge_info*)contact.o1->getUserData();
            auto d2 = (limb_edge_info*)contact.o2->getUserData();

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


bool collisionDetector::fixBound(double& x) {
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


void collisionDetector::lumelskyMinDist(int& idx1, int& idx2, int& idx3, int& idx4, int& idx5,
                                        int& idx6, double& dist, ConstraintType& constraint_type) {

    Vector3d x1s = soft_robots->limbs[idx5]->getVertex(idx1);
    Vector3d x1e = soft_robots->limbs[idx5]->getVertex(idx1+1);

    Vector3d x2s = soft_robots->limbs[idx6]->getVertex(idx2);
    Vector3d x2e = soft_robots->limbs[idx6]->getVertex(idx2+1);

    Vector3d e1 = x1e - x1s;
    Vector3d e2 = x2e - x2s;
    Vector3d e12 = x2s - x1s;

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
    if ((t == 0 || t == 1) && (uf == 0 || uf == 1)) // p2p
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
        constraint_type = PointToPoint;
    }
    else if (t == 0 || t == 1 || uf == 0 || uf == 1) //p2e
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
        constraint_type = PointToEdge;
    }
    else
    {
        idx3 = idx1 + 1;
        idx4 = idx2 + 1;
        constraint_type = EdgeToEdge;
    }
    dist = (e1 * t - e2 * uf - e12).norm();
}



void collisionDetector::narrowPhaseCollisionDetection() {
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
        double centerline_to_centerline_dist = soft_robots->limbs[idx5]->rod_radius + soft_robots->limbs[idx6]->rod_radius;
        double contact_limit = centerline_to_centerline_dist + delta;
        double numerical_limit = centerline_to_centerline_dist - delta;

        if (curr_dist - centerline_to_centerline_dist < min_dist) {
            min_dist = curr_dist - centerline_to_centerline_dist;
        }

        if (curr_dist < contact_limit) {
            contact_ids.emplace_back(idx1, idx2, idx3, idx4, idx5, idx6,
                                     constraint_type,
                                     (curr_dist > numerical_limit) ? NonPenetrated : Penetrated);
            num_collisions++;
        }
    }
}

