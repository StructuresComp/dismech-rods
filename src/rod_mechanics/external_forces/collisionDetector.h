#ifndef COLLISIONDETECTOR_H
#define COLLISIONDETECTOR_H

#include "eigenIncludes.h"
#include "elasticRod.h"
#include "contactEnums.h"


class collisionDetector
{
public:
    collisionDetector(shared_ptr<elasticRod> m_rod, double m_delta, double m_col_limit);

    bool constructCandidateSet(bool ignore_escape);
    void detectCollisions();

    MatrixXi edge_ids;
    MatrixXi contact_ids;
    vector<Vector2i> candidate_set;
    int num_collisions;
    double min_dist;

private:
    shared_ptr<elasticRod> rod = nullptr;
    double delta;
    double col_limit;
    int num_edge_combos;
    double scale;
    double contact_limit;
    double candidate_limit;
    double numerical_limit;
    double surface_limit;

    void fixbound(double &x);
    void computeMinDistance(const Vector3d &v1s, const Vector3d &v1e, const Vector3d &v2s, const Vector3d &v2e, double& dist);
    void computeMinDistance(int &idx1, int &idx2, int &idx3, int &idx4, double &dist, ConstraintType &constraint_type);
};

#endif