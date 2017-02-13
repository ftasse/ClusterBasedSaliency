#ifndef HEMESH_H
#define HEMESH_H

#include <map>
#include <set>
#include <vector>
#include <Eigen/Core>

class HEMesh
{
    typedef Eigen::Vector3d Vec3d;
    typedef int Id;

public:
    typedef Id VertexId;
    typedef Id HEdgeId;
    typedef Id FaceId;

    struct Base
    {
        Id idx;
        bool is_deleted;
    };

    struct Vertex : public Base
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Vec3d point;
        std::set<Id> in_hedges;
        std::set<Id> out_hedges;
        std::vector<int> record;

        Eigen::MatrixXd Q;
    };

    struct HEdge : public Base
    {
        Id verts[2];
        Id face;
        Id opp_he;

        int updateLevel;
    };

    struct Face : public Base
    {
        Id hedges[3];
    };

    Vertex* vertex(VertexId idx)
    {
        return &vertices[idx];
    }

    HEdge* hedge(HEdgeId idx)
    {
        return &hedges[idx];
    }

    Face* face(FaceId idx)
    {
        return &faces[idx];
    }

    //Iterators
    typedef std::vector<Vertex>::iterator VertexIter;
    typedef std::vector<Vertex>::const_iterator VertexConstIter;
    VertexIter vertices_begin() { return vertices.begin(); }
    VertexConstIter vertices_begin() const { return vertices.begin(); }
    VertexIter vertices_end() { return vertices.end(); }
    VertexConstIter vertices_end() const { return vertices.end(); }

    typedef std::vector<HEdge>::iterator HEdgeIter;
    typedef std::vector<HEdge>::const_iterator HEdgeConstIter;
    HEdgeIter hedges_begin() { return hedges.begin(); }
    HEdgeConstIter hedges_begin() const { return hedges.begin(); }
    HEdgeIter hedges_end() { return hedges.end(); }
    HEdgeConstIter hedges_end() const { return hedges.end(); }

    typedef std::vector<Face>::iterator FaceIter;
    typedef std::vector<Face>::const_iterator FaceConstIter;
    FaceIter faces_begin() { return faces.begin(); }
    FaceConstIter faces_begin() const { return faces.begin(); }
    FaceIter faces_end() { return faces.end(); }
    FaceConstIter faces_end() const { return faces.end(); }

    HEMesh()
    {

    }

    template <class PointCloudT, class PolygonT>
    HEMesh(const PointCloudT &cloud, const std::vector<PolygonT> &polygons)
    {
        for (unsigned int i = 0; i < cloud.points.size(); ++i)
        {
            Vec3d point(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
            add_vertex(point);
        }

        for (unsigned int i = 0; i < polygons.size(); ++i)
        {
            std::vector<int> indices;
            for (unsigned int j  = 0; j < polygons[i].vertices.size(); ++j)
            {
                indices.push_back( polygons[i].vertices[j]);
            }

            if (indices.size() == 3)
            {
                add_face((VertexId)indices[0], (VertexId)indices[1], (VertexId)indices[2]);
            }
        }
    }

    template <class PointCloudT, class PolygonT>
    void convertTo(PointCloudT &cloud, std::vector<PolygonT> &polygons)
    {
        cloud.points.clear();
        polygons.clear();

        /*for (VertexIter v_it=vertices_begin(); v_it!=vertices_end(); ++v_it)
        {
            if (!(*v_it).is_deleted)
                remove_duplicate_edges(v_it->idx);
        }*/

        cloud.points.resize(num_valid_vertices());
        polygons.resize(num_valid_faces());

		// std::cout << "Stats: " << cloud.points.size() << " " << polygons.size() << "\n";
        
        std::map<int, int> index_map;
        int k = 0;
        for (VertexIter vit = vertices_begin(); vit!=vertices_end(); ++vit)
        {
            if  (vit->is_deleted)   continue;
            index_map[vit->idx] = k;
            cloud.points[k].x = vit->point[0];
            cloud.points[k].y = vit->point[1];
            cloud.points[k].z = vit->point[2];
            ++k;
        }

        k = 0;
        for (FaceIter fit = faces_begin(); fit!=faces_end(); ++fit)
        {
            if  (fit->is_deleted)   continue;
            for (unsigned int i = 0; i < 3; ++i)
                polygons[k].vertices.push_back(index_map[hedge(fit->hedges[i])->verts[0]]);
            ++k;
        }
    }

    int num_hedges() {return hedges.size();}
    int num_vertices() {return vertices.size();}
    int num_faces() {return faces.size();}

    int num_valid_hedges()
    {
        int count = 0;
        for (HEdgeIter hit = hedges_begin(); hit != hedges_end(); ++hit)
            if (!hit->is_deleted)
                ++count;
        return count;
    }

    int num_valid_vertices()
    {
        int count = 0;
        for (VertexIter vit = vertices_begin(); vit != vertices_end(); ++vit)
            if (!vit->is_deleted)
                ++count;
        return count;
    }

    int num_valid_faces()
    {
        int count = 0;
        for (FaceIter fit = faces_begin(); fit != faces_end(); ++fit)
            if (!fit->is_deleted)
                ++count;
        return count;
    }

    VertexId add_vertex(Vec3d point)
    {
        vertices.push_back(Vertex());
        Vertex& v = vertices.back();
        v.idx = vertices.size()-1;
        v.is_deleted = false;
        v.point = point;

        return v.idx;
    }

    HEdgeId add_hedge(VertexId vi, VertexId vj)
    {
        hedges.push_back(HEdge());
        HEdge& he = hedges.back();
        he.idx = hedges.size()-1;
        he.is_deleted = false;
        he.verts[0] = vi;
        he.verts[1] = vj;
        he.face = -1;
        he.opp_he = -1;
        he.updateLevel = 0;

        vertex(vi)->out_hedges.insert(he.idx);
        vertex(vj)->in_hedges.insert(he.idx);
        for (std::set<HEdgeId>::iterator ih_it = vertex(vi)->in_hedges.begin(); ih_it!=vertex(vi)->in_hedges.end(); ++ih_it)
        {
            if (hedge(*ih_it)->verts[0]==vj && hedge(*ih_it)->verts[1]==vi) //opp_half_edge
            {
                he.opp_he = *ih_it;
                hedge(*ih_it)->opp_he = he.idx;
                break;
            }
        }
        return he.idx;
    }

    FaceId add_face(VertexId v1, VertexId v2, VertexId v3)
    {
        faces.push_back(Face());
        Face& f = faces.back();
        f.idx = faces.size()-1;
        f.is_deleted = false;

        int vids[3] = {v1, v2, v3};
        for (int k=0; k<3; ++k)
        {
            f.hedges[k] = add_hedge(vids[k], vids[(k+1)%3]);
            hedge(f.hedges[k])->face = f.idx;
        }
        return f.idx;
    }

    void remove_hedge(VertexId heid)
    {
        if (hedge(heid)->face>=0)
        {
            FaceId f = hedge(heid)->face;
            for (int k=0; k<3; ++k)
            {
                if (face(f)->hedges[k]>=0)
                    hedge(face(f)->hedges[k])->face = -1;
                face(f)->hedges[k] = -1;
            }
            face(f)->is_deleted = true;
        }
        if (vertex(hedge(heid)->verts[1])->in_hedges.erase(heid)!= 1)
            std::cout << "Error3!\n" << std::flush;
        if (vertex(hedge(heid)->verts[0])->out_hedges.erase(heid)!= 1)
            std::cout << "Error3!\n" << std::flush;

        if (vertex(hedge(heid)->verts[1])->in_hedges.size()==0 &&
                vertex(hedge(heid)->verts[1])->out_hedges.size()==0)
            vertex(hedge(heid)->verts[1])->is_deleted = true;

        if (vertex(hedge(heid)->verts[0])->in_hedges.size()==0 &&
                vertex(hedge(heid)->verts[0])->out_hedges.size()==0)
            vertex(hedge(heid)->verts[0])->is_deleted = true;

        hedge(heid)->is_deleted = true;
        HEdgeId opph = hedge(heid)->opp_he;
        if (opph >=0 )
        {
            hedge(opph)->opp_he = -1;
            hedge(heid)->opp_he = -1;
            remove_hedge(opph);
        }
    }

    void remove_duplicate_edges(VertexId j)
    {
        std::vector<HEdgeId> in, out;
        std::vector<HEdgeId> in_del, out_del;
        for (std::set<HEdgeId>::iterator ih_it = vertex(j)->in_hedges.begin(); ih_it!=vertex(j)->in_hedges.end(); ++ih_it)
        {
            in.push_back(*ih_it);
        }
        for (std::set<HEdgeId>::iterator oh_it = vertex(j)->out_hedges.begin(); oh_it!=vertex(j)->out_hedges.end(); ++oh_it)
        {
            out.push_back(*oh_it);
        }

        std::vector<bool> in_status(in.size(), false);
        for (int m=0; m<in.size(); ++m)
        {
            for (int n=m+1; n<in.size(); ++n)
            {
                if (!in_status[m] && !in_status[n] && hedge(in[m])->verts[0] == hedge(in[n])->verts[0])   //Duplicate
                {
                    if (hedge(in[m])->face < 0)
                    {
                        in_status[m] = true;
                        in_del.push_back(in[m]);
                    }
                    else //if (hedge(in[n])->face < 0)
                    {
                        in_status[n] = true;
                        in_del.push_back(in[n]);
                    }
                    if (hedge(in[m])->face >=0 && hedge(in[n])->face >= 0)
                        std::cout << "Error6!\n" << std::flush;
                }
            }
        }
        for (int l=0; l<in_del.size(); ++l)
            if (!hedge((in_del[l]))->is_deleted)
                remove_hedge(in_del[l]);

        std::vector<bool> out_status(out.size(), false);
        for (int m=0; m<out.size(); ++m)
            if (hedge(out[m])->is_deleted)
                out_status[m] = true;

        for (int m=0; m<out.size(); ++m)
        {
            for (int n=m+1; n<out.size(); ++n)
            {
                if (!out_status[m] && !out_status[n] && hedge(out[m])->verts[1] == hedge(out[n])->verts[1])   //Duplicate
                {
                    if (hedge(out[m])->face < 0)
                    {
                        out_status[m] = true;
                        out_del.push_back(out[m]);
                    }
                    else //if (hedge(out[n])->face < 0)
                    {
                        out_status[n] = true;
                        out_del.push_back(out[n]);
                    }
                    if (hedge(out[m])->face >=0 && hedge(out[n])->face >= 0)
                        std::cout << "Error6!\n" << std::flush;
                }
            }
        }
        for (int l=0; l<out_del.size(); ++l)
            if (!hedge((out_del[l]))->is_deleted)
                remove_hedge(out_del[l]);
    }

    void collapse_edge(int i, int j)
    {
        HEdge* the_hedge = 0;

        // if (vertex(i)->is_deleted || vertex(j)->is_deleted) return;
        
        for (std::set<HEdgeId>::iterator oh_it = vertex(i)->out_hedges.begin(); oh_it!=vertex(i)->out_hedges.end(); ++oh_it)
        {
            if (hedge(*oh_it)->verts[1] == j)
            {
                the_hedge = hedge(*oh_it); break;
            }
        }

        if (the_hedge)  collapse_edge(the_hedge->idx);
		else std::cout << "Edge not found: " << i << " " << j << " " <<  vertex(i)->is_deleted << " " << vertex(j)->is_deleted << "\n";
    }

    void collapse_edge(HEdgeId he_id)
    {

        VertexId i = hedge(he_id)->verts[0];
        VertexId j = hedge(he_id)->verts[1];

        if (vertex(i)->is_deleted) std::cout << "Error!\n" << std::flush;
        if (vertex(i)->in_hedges.size() != vertex(i)->out_hedges.size()) std::cout << "Error2!\n" << std::flush;

        //Redirect edges
        std::vector<HEdgeId> in, out;
        for (std::set<HEdgeId>::iterator ih_it = vertex(i)->in_hedges.begin(); ih_it!=vertex(i)->in_hedges.end(); ++ih_it)
        {
            if (hedge(*ih_it)->verts[0]!=j)
            {
                hedge(*ih_it)->verts[1] = j;
                vertex(j)->in_hedges.insert(*ih_it);
            } else
            {
                /*if (vertex(j)->out_hedges.find(*ih_it) == vertex(j)->out_hedges.end())
                    std::cout << "Error4!\n" << std::flush;
                else */  out.push_back(*ih_it);
            }
        }
        for (std::set<HEdgeId>::iterator oh_it = vertex(i)->out_hedges.begin(); oh_it!=vertex(i)->out_hedges.end(); ++oh_it)
        {
            if (hedge(*oh_it)->verts[1] != j)
            {
                hedge(*oh_it)->verts[0] = j;
                vertex(j)->out_hedges.insert(*oh_it);
            } else
            {
                /*if (vertex(j)->in_hedges.find(*oh_it) == vertex(j)->in_hedges.end())
                    std::cout << "Error4!\n" << std::flush;
                else*/    in.push_back(*oh_it);
            }
        }

        //Delete stuff
        for (int l=0; l<in.size(); ++l)
            if (!hedge(in[l])->is_deleted)
                remove_hedge(in[l]);
        for (int l=0; l<out.size(); ++l)
            if (!hedge(out[l])->is_deleted)
                remove_hedge(out[l]);

        vertex(i)->in_hedges.clear();
        vertex(i)->out_hedges.clear();
        vertex(i)->is_deleted = true;
        //remove_duplicate_edges(j);
    }

    void garbage_collection()
    {
        std::vector<int> rem_faces(num_faces(), -1),
                rem_hedges(num_hedges(),-1),
                rem_vertices(num_vertices(),-1);

        int counter = 0;
        for (int i=0; i<vertices.size();  ++i)
        {
            if (vertex(i)->is_deleted)
            {
                ++counter;
                rem_vertices[i] = -1;
            } else
            {
                rem_vertices[i] = i-counter;
            }
        }

        counter = 0;
        for (int i=0; i<hedges.size();  ++i)
        {
            if (hedge(i)->is_deleted)
            {
                ++counter;
                rem_hedges[i] = -1;
            } else
            {
                rem_hedges[i] = i-counter;
            }
        }

        counter = 0;
        for (int i=0; i<faces.size();  ++i)
        {
            if (face(i)->is_deleted)
            {
                ++counter;
                rem_faces[i] = -1;
            } else
            {
                rem_faces[i] = i-counter;
            }
        }

        for (int i = vertices.size()-1; i>=0; --i)
        {
            if (vertex(i)->is_deleted)
            {
                vertices.erase(vertices.begin()+i);
            } else
            {
                vertex(i)->idx = rem_vertices[vertex(i)->idx];
                std::set<int> in, out;
                for (std::set<HEdgeId>::iterator ih_it = vertex(i)->in_hedges.begin(); ih_it!=vertex(i)->in_hedges.end(); ++ih_it)
                {
                    in.insert(rem_hedges[*ih_it]);
                }
                vertex(i)->in_hedges = in;
                for (std::set<HEdgeId>::iterator oh_it = vertex(i)->out_hedges.begin(); oh_it!=vertex(i)->out_hedges.end(); ++oh_it)
                {
                    out.insert(rem_hedges[*oh_it]);
                }
                vertex(i)->out_hedges = out;
            }
        }

        for (int i = hedges.size()-1; i>=0; --i)
        {
            if (hedge(i)->is_deleted)
            {
                hedges.erase(hedges.begin()+i);
            } else
            {
                hedge(i)->idx = rem_hedges[hedge(i)->idx];
                hedge(i)->verts[0] = rem_vertices[hedge(i)->verts[0]];
                hedge(i)->verts[1] = rem_vertices[hedge(i)->verts[1]];
                if (hedge(i)->opp_he>=0) hedge(i)->opp_he = rem_hedges[hedge(i)->opp_he];
                if (hedge(i)->face>=0) hedge(i)->face = rem_faces[hedge(i)->face];
            }
        }

        for (int i = faces.size()-1; i>=0; --i)
        {
            if (face(i)->is_deleted)
            {
                faces.erase(faces.begin()+i);
            } else
            {
                face(i)->idx = rem_faces[face(i)->idx];
                for (int k=0; k<3; ++k)
                    if (face(i)->hedges[k]>=0)
                        face(i)->hedges[k] = rem_hedges[face(i)->hedges[k]];
            }
        }
    }
	
	void fix_topology()
	{
		int nh = num_valid_hedges();
	
		std::vector<int> valid_hedges(nh);
		int k=0;
		for (HEdgeIter hit = hedges_begin(); hit != hedges_end(); ++hit)
		{
			if (hit->is_deleted) continue;
			else	valid_hedges[k] = hit->idx;
			++k;
		}
		for (int i=0; i<nh; ++i)
		{
			if (hedge(valid_hedges[i])->opp_he < 0)
			{
				VertexId vi = hedge(valid_hedges[i])->verts[0];
				VertexId vj = hedge(valid_hedges[i])->verts[1];
				add_hedge(vj, vi);
				/*for (std::set<HEdgeId>::iterator ih_it = vertex(vi)->in_hedges.begin(); ih_it!=vertex(vi)->in_hedges.end(); ++ih_it)
				{
					if (hedge(*ih_it)->verts[0]==vj && hedge(*ih_it)->verts[1]==vi) //opp_half_edge
					{
						hedge(valid_hedges[i])->opp_he = *ih_it;
						hedge(*ih_it)->opp_he = hedge(valid_hedges[i])->idx;
						hedge(*ih_it)->is_deleted = false;
						break;
					}
				}*/
			}
		}
	}

private:
    std::vector<Vertex> vertices;
    std::vector<HEdge> hedges;
    std::vector<Face> faces;

};

#endif // HEMesh
