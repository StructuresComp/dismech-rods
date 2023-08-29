#include "utils.h"
#include <fstream>


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



void load_txt(const string& filename, vector<Vector3d>& data) {
    fstream cin;
    cin.open(filename);
    if (cin.fail()) {
        cout << "Failed to open file " << filename << endl;
        exit(1);
    }

    // TODO: add a check to see data is in proper structure (N, 3)
    string s;
    int number_of_lines = 0;
    while (getline(cin, s)) {
        number_of_lines++;
    }
    cin.close();

    cin.open(filename);
    Vector3d entry = Vector3d::Zero();
    int i = 0;
    while (i < number_of_lines) {
        cin >> s;
        entry(0) = stod(s);
        cin >> s;
        entry(1) = stod(s);
        cin >> s;
        entry(2) = stod(s);
        data.emplace_back(entry);
        i++;
    }
}
