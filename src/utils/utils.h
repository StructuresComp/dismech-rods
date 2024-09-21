#ifndef UTILS_H
#define UTILS_H

#include "globalDefinitions.h"
#include <fstream>

string convert_float_to_scientific_str(double d);

// This function acts like np.loadtxt from Python
template <class VecN>
void load_txt(const string& filename, vector<VecN>& data);

#endif
