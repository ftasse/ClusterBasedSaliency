#include "hks.h"

#include "eigendecomposition.h"
#include "generate_lpmatrix.h"

#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
typedef OpenMesh::TriMesh_ArrayKernelT<> Mesh;

// #include <OpenMesh/Core/IO/MeshIO.hh>
// #include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
// typedef OpenMesh::PolyMesh_ArrayKernelT<> PMesh;

#include <fstream>

template<typename Mesh>
Eigen::SparseMatrix<double> compute_spectra(Mesh &mesh, Eigen::VectorXd &eigvals,
                            Eigen::MatrixXd &eigvecs, int nev)
{
    Eigen::SparseMatrix<double> W, A;

    compute_laplacian_matrix_Pinkall_Polthier(mesh, W, A);
    compute_eigen_decomp_arpack(W, A, eigvals, eigvecs, nev);
    std::ofstream ofs;//("W.txt");
    // ofs << W << "\n";
    // ofs.close();
    ofs.open("A.txt");
    ofs << A.diagonal() << "\n";
    ofs.close();
    std::cout << eigvals.size() << "th eigenvalue: " << eigvals(eigvals.size()-1) << "\n" << std::flush;

    return A;
}

template<typename Mesh>
Eigen::MatrixXd compute_hks(Mesh &mesh, int nev, int nstep, bool scale)
{
    Eigen::VectorXd eigvals;
    Eigen::MatrixXd eigvecs;

    Eigen::SparseMatrix<double> A = compute_spectra(mesh, eigvals, eigvecs, nev);
    Eigen::MatrixXd hks = compute_hks(eigvals, eigvecs, nstep);

    if (scale)
    {
        Eigen::VectorXd scale = ((A*hks).colwise().sum()).array().pow(-1); // 1.0/(A*hks)
        hks = hks * scale.asDiagonal();
    }

    // std::cout << "HKS: \n" << hks.block(0, 0, 5, 5) << "\n" << std::flush;
    // std::cout << hks.rows() <<" " << hks.cols() << "\n";
    return hks;
}

Eigen::MatrixXd compute_hks(const Eigen::MatrixX3f& points, const Eigen::MatrixX3i& polygons, int nev, int nstep, bool scale) {
	Mesh oMesh;
  oMesh.clear();
  // printf("%d %d %d %d\n", points.rows(), points.cols(), polygons.rows(), polygons.cols());
  for(size_t i = 0; i < points.rows(); ++i) {
    oMesh.add_vertex(Mesh::Point(points(i, 0), points(i, 1), points(i, 2)));
  }
  if (polygons.rows() > 0 && polygons.cols() == 3)
	  for(size_t i = 0; i < polygons.rows(); ++i) {
	    std::vector<Mesh::VertexHandle> vhandles;
	    for (size_t k = 0; k < polygons.cols(); ++k) {
	    	// printf("%d from %d\n", polygons(i, k), oMesh.n_vertices());
	    	vhandles.push_back(oMesh.vertex_handle(polygons(i, k)));
	    }
	    // printf("add face\n");
	    oMesh.add_face(vhandles);
	    // printf("added face\n");
	  }

  return compute_hks(oMesh, nev, nstep, scale);
}

