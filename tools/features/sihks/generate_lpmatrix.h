#ifndef GENERATE_LPMATRIX_H
#define GENERATE_LPMATRIX_H

#include <float.h>
#include <Eigen/Sparse>
#include <OpenMesh/Core/Geometry/MathDefs.hh>

enum Metric
{
    EuclideanMetric = 0,
    GeodesicMetric = 1
};

template<class MeshT>
void compute_one2part_Euclidean_vdist(unsigned int vid_start, MeshT& mesh,
                                      std::vector< std::pair<unsigned int, double> >& vgdists,
                                      double maxdist)
{
    typename MeshT::VertexHandle vh = mesh.vertex_handle(vid_start);
    for(unsigned int j = 0; j < mesh.n_vertices(); j ++)
    {
        typename MeshT::VertexHandle wh = mesh.vertex_handle(j);
        double d = sqrt( dot(mesh.point(wh) - mesh.point(vh), mesh.point(wh) - mesh.point(vh)) );
        if(d <= maxdist){
            vgdists.push_back( std::make_pair(j, d) );
        }
    }
}
/* from OpenFlipper ACG Geometry */
template<class VectorT>
int isObtuse(const VectorT& _p0,
             const VectorT& _p1,
             const VectorT& _p2 )
{
    const double a0 = ((_p1-_p0)|(_p2-_p0));

    if ( a0<0.0) return 1;
    else
    {
        const double a1 = ((_p0-_p1)|(_p2-_p1));

        if (a1 < 0.0 ) return 2;
        else
        {
            const double a2 = ((_p0-_p2)|(_p1-_p2));

            if (a2 < 0.0 ) return 3;
            else return 0;
        }
    }
}

template<class MeshT, class SparseMatrix>
void compute_laplacian_matrix_Pinkall_Polthier(MeshT &mesh, SparseMatrix &_W, SparseMatrix &_D,
                                               Metric metric = EuclideanMetric, double h = DBL_MAX,
                                               double rho = 3.0 )
{
    typedef Eigen::Triplet<double> Triplet;

    int n = mesh.n_vertices();
    _W = SparseMatrix(n,n);
    _D = SparseMatrix(n,n);
    std::vector<Triplet> wTriplets; wTriplets.reserve(7*n);
    std::vector<Triplet> dTriplets; dTriplets.reserve(n);

    typename MeshT::VertexIter v_it;
    for (v_it= mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
    {
        typedef typename MeshT::Scalar REALT;
        double _weight = 0.0;
        double _area = 0.0;

        typename MeshT::ConstVertexOHalfedgeIter voh_it = mesh.cvoh_iter(*v_it);
        if ( ! voh_it->is_valid() ) continue;

        for(; voh_it.is_valid(); ++voh_it)
        {   
            if ( mesh.is_boundary( mesh.edge_handle( *voh_it ) ) )
                continue;

            const typename MeshT::Point p0 = mesh.point( *v_it );
            const typename MeshT::Point p1 = mesh.point( mesh.to_vertex_handle( *voh_it));
            const typename MeshT::Point p2 = mesh.point( mesh.from_vertex_handle( mesh.prev_halfedge_handle( *voh_it)));
            const typename MeshT::Point p3 = mesh.point( mesh.to_vertex_handle( mesh.next_halfedge_handle( mesh.opposite_halfedge_handle(*voh_it))));

            const REALT alpha = acos( OpenMesh::sane_aarg((p0-p2).normalize() | (p1-p2).normalize()) );
            const REALT beta  = acos( OpenMesh::sane_aarg((p0-p3).normalize() | (p1-p3).normalize()) );

            REALT cotw = 0.0;

            if ( !OpenMesh::is_eq(alpha,M_PI/2.0) ) cotw += (REALT(1.0))/tan(alpha);
            if ( !OpenMesh::is_eq(beta,M_PI/2.0) ) cotw += (REALT(1.0))/tan(beta);
            if ( OpenMesh::is_zero(cotw) || isinf(cotw) ) continue;

            // calculate voronoi area
            const int obt = isObtuse(p0,p1,p2);
            if(obt == 0)
            {
                REALT gamma = acos( OpenMesh::sane_aarg((p0-p1).normalize() | (p2-p1).normalize()) );

                REALT tmp = 0.0;
                if ( !OpenMesh::is_eq(alpha,M_PI/2.0) ) tmp += (p0-p1).sqrnorm()*1.0/tan(alpha);
                if ( !OpenMesh::is_eq(gamma,M_PI/2.0) ) tmp += (p0-p2).sqrnorm()*1.0/tan(gamma);
                if ( OpenMesh::is_zero(tmp) || isinf(tmp) ) continue;
                _area += 0.125*( tmp );
            }
            else
            {
                if(obt == 1) _area += ((p1-p0) % (p2-p0)).norm() * 0.5 * 0.5;
                else _area += ((p1-p0) % (p2-p0)).norm() * 0.5 * 0.25;
            }

            wTriplets.push_back(Triplet(v_it->idx(),
                                        mesh.to_vertex_handle(*voh_it).idx(), cotw));
            _weight += cotw;

        }

        wTriplets.push_back(Triplet(v_it->idx(), v_it->idx(), -_weight));
        dTriplets.push_back(Triplet(v_it->idx(), v_it->idx(), 2*_area));
    }

    _W.setFromTriplets(wTriplets.begin(), wTriplets.end());
    _D.setFromTriplets(dTriplets.begin(), dTriplets.end());
}


template<class MeshT, class SparseMatrix>
void compute_dual_laplacian_matrix_Tutte(MeshT &mesh, SparseMatrix &_Wd)
{
    typedef Eigen::Triplet<double> Triplet;

    int n = mesh.n_faces();
    _Wd = SparseMatrix(n,n);
    std::vector<Triplet> wTriplets; wTriplets.reserve(4*n);

    typename MeshT::FaceIter f_it;
    for (f_it= mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it)
    {
        typedef typename MeshT::Scalar REALT;
        double _weight = 0.0;

        typename MeshT::ConstFaceHalfedgeIter fh_it = mesh.cfh_iter(f_it);
        if ( ! fh_it->is_valid() ) continue;

        for(; fh_it; ++fh_it)
        {
            wTriplets.push_back(Triplet(f_it->idx(),
                                        mesh.face_handle(mesh.opposite_halfedge_handle(fh_it.handle())).idx(),
                                        1.0));
            _weight += 1.0;
        }
        wTriplets.push_back(Triplet(f_it->idx(), f_it->idx(), -_weight));
    }
    _Wd.setFromTriplets(wTriplets.begin(), wTriplets.end());
}


template<class MeshT, class SparseMatrix>
void compute_dual_laplacian_matrix_Pinkall_Polthier(MeshT &mesh, SparseMatrix &_Wd,
                                               Metric metric = EuclideanMetric, double h = DBL_MAX,
                                               double rho = 3.0 )
{
    typedef Eigen::Triplet<double> Triplet;

    int n = mesh.n_faces();
    _Wd = SparseMatrix(n,n);
    std::vector<Triplet> wTriplets; wTriplets.reserve(4*n);

    typename MeshT::FaceIter f_it;
    for (f_it= mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it)
    {
        typedef typename MeshT::Scalar REALT;
        double _weight = 0.0;

        typename MeshT::ConstFaceHalfedgeIter fh_it = mesh.cfh_iter(f_it);
        if ( ! fh_it->is_valid() ) continue;

        for(; fh_it; ++fh_it)
        {
            if ( mesh.is_boundary( mesh.edge_handle( fh_it.handle() ) ) )
                continue;
            const typename MeshT::Point p0 = mesh.point( mesh.from_vertex_handle( fh_it.handle()));
            const typename MeshT::Point p1 = mesh.point( mesh.to_vertex_handle( fh_it.handle()));
            const typename MeshT::Point p2 = mesh.point( mesh.from_vertex_handle( mesh.prev_halfedge_handle( fh_it.handle())));
            const typename MeshT::Point p3 = mesh.point( mesh.to_vertex_handle( mesh.next_halfedge_handle( mesh.opposite_halfedge_handle(fh_it.handle()))));

            const REALT alpha = acos( OpenMesh::sane_aarg((p0-p2).normalize() | (p1-p2).normalize()) );
            const REALT beta  = acos( OpenMesh::sane_aarg((p0-p3).normalize() | (p1-p3).normalize()) );
            REALT cotw = 0.0;

            if ( !OpenMesh::is_eq(alpha,M_PI/2.0) ) cotw += (REALT(1.0))/tan(alpha);
            if ( !OpenMesh::is_eq(beta,M_PI/2.0) ) cotw += (REALT(1.0))/tan(beta);
            if ( OpenMesh::is_zero(cotw) || isinf(cotw) ) continue;

            wTriplets.push_back(Triplet(f_it->idx(),
                                        mesh.face_handle(mesh.opposite_halfedge_handle(fh_it.handle())).idx(),
                                        cotw));
            _weight += cotw;
        }

        wTriplets.push_back(Triplet(f_it->idx(), f_it->idx(), -_weight));
    }

    _Wd.setFromTriplets(wTriplets.begin(), wTriplets.end());
}

template<class MeshT, class SparseMatrix>
void compute_laplacian_matrix_Xu_Meyer(MeshT &mesh, SparseMatrix &_W, SparseMatrix &_D,
                                       Metric metric = EuclideanMetric,
                                       double h = DBL_MAX, double rho = 3.0 )
{

}

template<class MeshT, class SparseMatrix>
void compute_laplacian_matrix_Belkin(MeshT &mesh, SparseMatrix &_W, SparseMatrix &_D,
                                     Metric metric = EuclideanMetric,
                                     double h = DBL_MAX, double rho = 3.0 )
{

}

#endif
