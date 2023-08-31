#ifndef UTILS_H
#define UTILS_H

#include "eigenIncludes.h"
#include "rod_mechanics/elasticRod.h"
#include <fstream>


void lock_edge(const shared_ptr<elasticRod>& limb, int edge_num);
void apply_initial_velocities(const shared_ptr<elasticRod>& limb, vector<Vector3d>& velocities);

// This function acts like np.loadtxt from Python
template <class VecN>
void load_txt(const string& filename, vector<VecN>& data);

#endif