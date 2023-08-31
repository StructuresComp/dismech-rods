#include "utils.h"


void lock_edge(const shared_ptr<elasticRod>& limb, int edge_num)
{
    limb->setVertexBoundaryCondition(limb->getVertex(edge_num), edge_num);
    limb->setVertexBoundaryCondition(limb->getVertex(edge_num+1), edge_num+1);
    limb->setThetaBoundaryCondition(0.0, edge_num);
}


void apply_initial_velocities(const shared_ptr<elasticRod>& limb, vector<Vector3d>& velocities) {
    if (limb->nv != velocities.size()) {
        throw runtime_error("The number of nodes (" + to_string(limb->nv) +
                            ") and velocities (" + to_string(velocities.size()) + ") given did not match!");
    }
    for (int i = 0; i < limb->nv; i++) {
        limb->u.segment(4*i, 3) = limb->u0.segment(4*i, 3) = velocities.at(i);
    }
}


template <class VecN>
void load_txt(const string& filename, vector<VecN>& data) {
    fstream cin;
    cin.open(filename);
    if (cin.fail()) {
        cout << "Failed to open file " << filename << endl;
        exit(1);
    }

    // TODO: add a check to see data is in proper (N, M) structure
    string s;
    int number_of_lines = 0;
    while (getline(cin, s)) {
        number_of_lines++;
    }
    cin.close();

    cin.open(filename);
    VecN entry = VecN::Zero();
    int i = 0;
    int j;
    int shape = entry.size();
    while (i < number_of_lines) {
        j = 0;
        while (j < shape) {
            cin >> s;
            entry(j) = stod(s);
            j++;
        }
        data.emplace_back(entry);
        i++;
    }
}

template void load_txt<Vector3d>(const string& filename, vector<Vector3d>& data);
template void load_txt<Vector4d>(const string& filename, vector<Vector4d>& data);
