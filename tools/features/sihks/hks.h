#ifndef SIHKS_H
#define SIHKS_H

#include <Eigen/Eigen>
#include <Eigen/Sparse>

#include <iostream>

template<typename Mesh>
Eigen::SparseMatrix<double> compute_spectra(Mesh &mesh, Eigen::VectorXd &eigvals,
                            Eigen::MatrixXd &eigvecs, int nev);

template<typename Mesh>
Eigen::MatrixXd compute_hks(Mesh &mesh, int nev, int nstep = 100, bool scale = true);

Eigen::MatrixXd compute_hks(const Eigen::MatrixX3f& points, const Eigen::MatrixX3i& polygons, int nev, int nstep = 100, bool scale = true);



template<typename Matrix>
Matrix compute_hks(Eigen::VectorXd &eigvals,
                            Matrix &eigvecs, int nstep = 100)
{
    double maxeigv = eigvals(eigvals.size()-1);
    double mineigv = eigvals(1);
    double tmin = std::abs(4*log(10) / maxeigv);
    double tmax = std::abs(4*log(10) / mineigv);

    double ltmin = log(tmin), ltmax = log(tmax);
    double stepsize = (ltmax - ltmin) / nstep;
    Eigen::RowVectorXd ts(nstep+1);
    for (unsigned int i=0; i<ts.cols(); ++i)
        ts(i) = exp( ltmin + i*stepsize);

    //|psi[k]|^2 for k>0
    Matrix T1 = eigvecs.block(0, 1, eigvecs.rows(), eigvecs.cols()-1).array().pow(2);
    //exp(-|lamda[k]|*t) for k>0
    Matrix T2 = (- eigvals.block(1, 0, eigvals.rows()-1, eigvals.cols()).array().abs().matrix() * ts).array().exp();

    Matrix hks = (T1*T2).array().abs();
    return hks;
}


#endif //SIKHS_H


