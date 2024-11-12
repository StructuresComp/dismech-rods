#ifndef UTILS_H
#define UTILS_H

#include "global_definitions.h"

std::string convertFloatToScientificStr(double d);

// This function acts like np.loadtxt from Python
template <class VecN>
void loadTxt(const std::string& filename, std::vector<VecN>& data);

#endif
