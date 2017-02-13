#include <Eigen/Cholesky>
#include <Eigen/Sparse>


// #include <Eigen/UmfPackSupport>
// #include <Eigen/SuperLUSupport>
// #include <Eigen/CholmodSupport>
#include "Eigen3ArpackSupport/ArpackSupport"

#include "eigendecomposition.h"

typedef Eigen::SparseMatrix<double> Matrix;
typedef Eigen::SimplicialLDLT<Matrix> LinearSolver;
// typedef Eigen::CholmodSupernodalLLT<Matrix> LinearSolver;
// typdef Eigen::UmfPackLU<Matrix> LinearSolver;
// typedef Eigen::SuperLU<Matrix> LinearSolver;

void compute_eigen_decomp_arpack(Eigen::SparseMatrix<double> &_A,
                                 Eigen::VectorXd &eigvals,
                                 Eigen::MatrixXd &eigvecs,
                                 int nb_eigs)
{
    Eigen::ArpackGeneralizedSelfAdjointEigenSolver<Matrix, LinearSolver > eig_solver;
    eig_solver.compute(_A, nb_eigs, "SM", Eigen::ComputeEigenvectors);
    eigvals = eig_solver.eigenvalues();
    eigvecs = eig_solver.eigenvectors();

    eigvals.reverseInPlace();
    for (int i=0; i<eigvecs.rows(); ++i)
        eigvecs.row(i).reverseInPlace();
}

void compute_eigen_decomp_arpack(Eigen::MatrixXf &_A,
                                 Eigen::VectorXf &eigvals,
                                 Eigen::MatrixXf &eigvecs,
                                 int nb_eigs, const char *mode) {
    Eigen::ArpackGeneralizedSelfAdjointEigenSolver<Eigen::MatrixXf, Eigen::LDLT<Eigen::MatrixXf> > eig_solver;
    eig_solver.compute(_A, nb_eigs, mode, Eigen::ComputeEigenvectors);
    eigvals = eig_solver.eigenvalues();
    eigvecs = eig_solver.eigenvectors();

    if (std::string("SM") == mode) {
        eigvals.reverseInPlace();
        for (int i=0; i<eigvecs.rows(); ++i)
            eigvecs.row(i).reverseInPlace();  
    } 
}

void compute_eigen_decomp_arpack(Eigen::SparseMatrix<double> &_A,
                                 Eigen::SparseMatrix<double> &_B,
                                 Eigen::VectorXd &eigvals,
                                 Eigen::MatrixXd &eigvecs,
                                 int nb_eigs)
{
    Eigen::ArpackGeneralizedSelfAdjointEigenSolver<Matrix, LinearSolver > eig_solver;
    eig_solver.compute(_A, _B, nb_eigs, "SM", Eigen::ComputeEigenvectors);
    eigvals = eig_solver.eigenvalues();
    eigvecs = eig_solver.eigenvectors();

    eigvals.reverseInPlace();
    for (int i=0; i<eigvecs.rows(); ++i)
        eigvecs.row(i).reverseInPlace();
}
