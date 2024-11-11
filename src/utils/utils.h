#ifndef UTILS_H
#define UTILS_H

#include "global_definitions.h"
#include <fstream>

string convertFloatToScientificStr(double d);

// This function acts like np.loadtxt from Python
template <class VecN>
void loadTxt(const string& filename, vector<VecN>& data);

#endif
