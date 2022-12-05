#define EIGEN_USE_MKL_ALL
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Sparse>

#include <memory>
#include <unistd.h>
#include "math.h"
#include <stdlib.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/stat.h>        /* For mode constants */
#include <fcntl.h>           /* For O_* constants */

using namespace std;
using namespace Eigen;


//// TODO: find a better place for this later
//// This should all the relevant info for a joint
//struct Joint {
//    Vector3d x;
//    int joint_node;
//    vector<pair<int, int>> connected_nodes;  // node_number and limb_idx
//
//    // Add things like RefLen, material frames
//};



