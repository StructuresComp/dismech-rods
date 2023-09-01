#ifndef UTILS_H
#define UTILS_H

#include "eigenIncludes.h"
#include <fstream>


// This function acts like np.loadtxt from Python
template <class VecN>
void load_txt(const string& filename, vector<VecN>& data);

#endif