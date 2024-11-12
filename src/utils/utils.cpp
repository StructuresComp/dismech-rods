#include "utils.h"
#include <fstream>  // std::fstream
#include <iomanip>  // std::setprecision

std::string convertFloatToScientificStr(double d) {
    std::stringstream ss;
    ss << std::fixed << std::scientific << std::setprecision(2);
    ss << d;
    return ss.str();
}

template <class VecX>
void loadTxt(const std::string& filename, std::vector<VecX>& data) {
    std::fstream cin;
    cin.open(filename);
    if (cin.fail()) {
        std::cout << "Failed to open file " << filename << std::endl;
        exit(1);
    }

    // TODO: add a check to see data is in proper (N, M) structure
    std::string s;
    int number_of_lines = 0;
    while (getline(cin, s)) {
        number_of_lines++;
    }
    cin.close();

    cin.open(filename);
    VecX entry = VecX::Zero();
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

template void loadTxt<Vec3>(const std::string& filename, std::vector<Vec3>& data);
template void loadTxt<Vec4>(const std::string& filename, std::vector<Vec4>& data);
