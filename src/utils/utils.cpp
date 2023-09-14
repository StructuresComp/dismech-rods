#include "utils.h"
#include <iomanip>      // std::setprecision


string convert_float_to_scientific_str(double d)
{
    std::stringstream ss;
    ss << std::fixed << scientific << setprecision(2); // I know the precision, so this is fine
    ss << d;
    return ss.str();
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
