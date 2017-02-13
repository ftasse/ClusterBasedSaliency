#ifndef MATRIX_IO_H
#define MATRIX_IO_H

#include <fstream>
#include <limits>
#include <Eigen/Core>

template<typename M>
bool saveEigenMatrix(std::string filename, const M &matrix, bool append = false)
{
    std::ofstream ofs;
    if (!append) {
        ofs.open(filename.c_str(), std::ofstream::out);
    } else {
        ofs.open(filename.c_str(), std::ofstream::app);
        ofs << "\n";
    }
    ofs << matrix.format(Eigen::IOFormat(Eigen::FullPrecision, 0, " ", "\n", "", "", "", ""));
    ofs.close();
    return true;
}

template<typename M>
bool loadEigenMatrix(std::string filename, M &matrix)
{
    std::ifstream ifs(filename.c_str());

    if (!ifs.is_open()) {
        return false;
    }
    else
    {
        std::vector<typename M::Scalar> values;
        int line_count = 0;

        std::string line;
        while (std::getline(ifs, line)) {
            if (line.empty()) {
                continue;
            }
            else {
                ++line_count;
                std::istringstream tmp(line);
                std::string word;
                while(tmp >> word) {
                    if (word == "nan" || word == "inf") {
                        values.push_back(std::numeric_limits<typename M::Scalar>::max());
                    } else {
                        typename M::Scalar val;
                        std::istringstream(word) >> val;
                        values.push_back(val);
                    }
                }
            }
        }

        if (line_count == 0)    return  false;

        int col_count = values.size() / line_count;
        if (!M::IsRowMajor){
            matrix = Eigen::Map<M>(&values[0], col_count, line_count);
            matrix.transposeInPlace();
        } else {
            matrix = Eigen::Map<M>(&values[0], line_count, col_count);
        }
    }

    ifs.close();
    return true;
}

#endif // MATRIX_IO_H