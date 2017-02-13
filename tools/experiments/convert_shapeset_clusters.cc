#include <map>
#include <pcl/common/pca.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PolygonMesh.h>
#include <pcl/segmentation/supervoxel_clustering.h>

#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>

#include <pcl/common/distances.h>
#include <pcl/features/boost.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/johnson_all_pairs_shortest.hpp>

#include <tools/io/mesh_io.h>

#include <unsupported/Eigen/Splines>
#include <Eigen/SVD>

// http://xiangze.hatenablog.com/entry/20111218/1324223652
void mds_eigen(Eigen::MatrixXf & d, Eigen::MatrixXf & out){
  using namespace Eigen;
  MatrixXf d2(d.rows(),d.cols());
  MatrixXf nm(d.rows(),d.cols());

  for(int i=0;i<d.rows();i++)
    for(int j=0;j<d.cols();j++)
      d2(i,j)=d(i,j)*d(i,j);

  nm.setConstant(-1./d.rows());

  for(int i=0;i<d.rows();i++)
    nm(i,i)+=1.;

  MatrixXf  inm=nm * d2* nm.transpose();
  inm/=2;

  JacobiSVD<MatrixXf> svd(inm, ComputeThinU | ComputeThinV);
  MatrixXf sig=svd.singularValues();
  MatrixXf u=svd.matrixU();

  // out=u*sig;
  for(int i=0;i<out.rows();i++)
    for(int j=0;j<out.cols();j++)
      out(i,j)=u(i,j)*sqrt(sig(j));
}

void centerAndScale(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid (*cloud, centroid);
  pcl::demeanPointCloud (*cloud, centroid, *cloud);
  float max_sqr_distance = 0, d;
  pcl::PointXYZ cog (0, 0, 0);
  for (size_t i = 0; i < cloud->points.size (); ++i) {
      d = pcl::squaredEuclideanDistance(cog, cloud->points[i]);
      if (d > max_sqr_distance) max_sqr_distance = d;
  }
  float scale_factor = 1.0f / (sqrt(max_sqr_distance));
  Eigen::Affine3f matrix = Eigen::Affine3f::Identity();
  matrix.scale (scale_factor);
  pcl::transformPointCloud (*cloud, *cloud, matrix);
}

void representMeshAsClusterGraph(
    pcl::PolygonMesh& mesh, 
    std::map<uint32_t, std::vector<uint32_t> >& clusters,
    std::multimap<uint32_t, uint32_t>& cluster_adjacency
  ) 
{
  typedef pcl::PointXYZ PointT;
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
  centerAndScale(cloud);

  float voxel_resolution = 0.008f;
  float seed_resolution = 0.1f;
  float color_importance = 0.0f; // 0.2f;
  float spatial_importance = 0.4f;
  float normal_importance = 1.0f;

  pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution, false);
  // super.setUseSingleCameraTransform (false);
  super.setInputCloud (cloud);
  super.setColorImportance (color_importance);
  super.setSpatialImportance (spatial_importance);
  super.setNormalImportance (normal_importance);

  std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;
  pcl::console::print_highlight ("Extracting supervoxels!\n");
  super.extract (supervoxel_clusters);
  for (std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr >::iterator it = supervoxel_clusters.begin();
       it != supervoxel_clusters.end(); ++it)
    clusters[it->first] = std::vector<uint32_t>();
  pcl::console::print_info ("Found %d supervoxels\n", supervoxel_clusters.size ());

  pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud =  super.getLabeledCloud ();
  for (size_t i = 0; i < labeled_cloud->size(); ++i) {
    std::map<uint32_t, std::vector<uint32_t> >::iterator it = clusters.find(labeled_cloud->points[i].label);
    if (it != clusters.end()) it->second.push_back(i);
  }
 
  pcl::PointCloud<pcl::PointXYZRGBA >::Ptr colored_cloud = super.getColoredVoxelCloud ();
  // pcl::io::savePLYFile("clustered.ply", *colored_cloud);

  pcl::console::print_highlight ("Getting supervoxel adjacency\n");
  super.getSupervoxelAdjacency (cluster_adjacency);
  pcl::console::print_highlight ("Found %d connections\n", cluster_adjacency.size());
}

void embed3DPatchInPlane(
    pcl::PointCloud<pcl::PointXYZ>::Ptr& small_cloud,
    Eigen::MatrixXf& mds_points) 
{
  int num_points_per_patch = small_cloud->size();
  centerAndScale(small_cloud);
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud(small_cloud);
  pca.project(*small_cloud, *small_cloud);

  Eigen::MatrixXf patch(num_points_per_patch, 3);
  for (size_t i = 0; i < num_points_per_patch; ++i) 
    patch.row(i) = small_cloud->points[i].getVector3fMap();

  Eigen::MatrixXf dissimilarities(patch.rows(), patch.rows()); // TODO change to geodesic
  // for (int i = 0; i < dissimilarities.rows(); ++i)
  //   dissimilarities.row(i) = (patch.rowwise()-patch.row(i)).array().square().colwise().sum();

  using namespace boost;
  typedef property<edge_weight_t, float> Weight;
  typedef adjacency_list<vecS, vecS, undirectedS, no_property, Weight> Graph;
  Graph cloud_graph;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (small_cloud);
  for (size_t point_i = 0; point_i < small_cloud->points.size (); ++point_i) {
    std::vector<int> k_indices (9);
    std::vector<float> k_distances (9);
    kdtree.nearestKSearch (static_cast<int> (point_i), 9, k_indices, k_distances);
    for (int k_i = 0; k_i < static_cast<int> (k_indices.size ()); ++k_i)
      add_edge (point_i, k_indices[k_i], Weight (sqrtf (k_distances[k_i])), cloud_graph);
  }
  const size_t E = num_edges (cloud_graph), V = num_vertices (cloud_graph);
  // PCL_INFO ("The graph has %lu vertices and %lu edges.\n", V, E);
  std::vector< std::vector<float> > geodesic_distances_;
  for (size_t i = 0; i < V; ++i) {
    std::vector<float> aux (V);
    geodesic_distances_.push_back (aux);
  }
  johnson_all_pairs_shortest_paths (cloud_graph, geodesic_distances_);
  for (size_t point_i = 0; point_i < small_cloud->points.size (); ++point_i) {
    for (size_t point_j = 0; point_j < small_cloud->points.size (); ++point_j) {
      dissimilarities(point_i, point_j) = geodesic_distances_[point_i][point_j];
    }
  }
  
  mds_points.resize(patch.rows(), 2);
  mds_eigen(dissimilarities, mds_points);

  /*typedef Eigen::Spline<float, 3, Eigen::Dynamic> Spline3f;
    typedef Spline3f::PointType PointType;
    typedef Spline3f::KnotVectorType KnotVectorType;
    typedef Spline3f::ControlPointVectorType ControlPointVectorType;
   
    Spline3f fitted_surf;
    ControlPointVectorType points = patch.transpose();
    KnotVectorType chord_lengths; // knot parameters
    Eigen::ChordLengths(points, chord_lengths);
    const Spline3f spline = Eigen::SplineFitting<Spline3f>::Interpolate(points,3);  
    // interpolation without knot parameters
    {
      // const Spline2d spline = SplineFitting<Spline2d>::Interpolate(points,3,chord_lengths); 
      for (Eigen::DenseIndex i=0; i<points.cols(); ++i)
      {
        PointType pt = spline( chord_lengths(i) );
        PointType ref = points.col(i);
        assert( (pt - ref).matrix().norm() < 1e-14 );
      }
    }
    patch = spline.ctrls();*/
}

void convert2DEmbeddingToGrid(const Eigen::MatrixXf& mds_points, Eigen::MatrixXf& grid_2d, int width, int height) 
{
  float min_x = mds_points.col(1).minCoeff();
  float min_y = mds_points.col(0).minCoeff();
  float max_x = mds_points.col(1).maxCoeff();
  float max_y = mds_points.col(0).maxCoeff();

  grid_2d = Eigen::MatrixXf::Zero(height, width);
  for (int i = 0; i < mds_points.rows(); ++i) {
    float y = (mds_points(i,0) - min_y)*(height-1)/(max_y-min_y);
    float x = (mds_points(i,1) - min_x)*(width-1)/(max_x-min_x);
    grid_2d(y, x) = 1; // or 3d coordinates of point i
  }
}

void convert2DEmbeddingToCoordinateGrids(
  pcl::PointCloud<pcl::PointXYZ>::Ptr& small_cloud,
    const Eigen::MatrixXf& mds_points, 
    std::vector<Eigen::MatrixXf>& grids_2d, int width, int height) 
{
  float min_x = mds_points.col(1).minCoeff();
  float min_y = mds_points.col(0).minCoeff();
  float max_x = mds_points.col(1).maxCoeff();
  float max_y = mds_points.col(0).maxCoeff();

  grids_2d =  std::vector<Eigen::MatrixXf>(3, Eigen::MatrixXf::Zero(width, height));
  for (int i = 0; i < mds_points.rows(); ++i) {
    float y = (mds_points(i,0) - min_y)*(height-1)/(max_y-min_y);
    float x = (mds_points(i,1) - min_x)*(width-1)/(max_x-min_x);
    for (int k = 0; k < 3; ++k)
      grids_2d[k](y, x) = small_cloud->points[i].getVector3fMap()[k];
  }
}

void representMeshAsClusterGraph(
    pcl::PolygonMesh& mesh, 
    std::map<uint32_t, Eigen::MatrixXf>& patches,
    std::multimap<uint32_t, uint32_t>& cluster_adjacency,
    int num_points_per_patch) 
{
  std::map<uint32_t, std::vector<uint32_t> > clusters;
  representMeshAsClusterGraph(mesh, clusters, cluster_adjacency);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
  std::vector<int> vertex_faces(cloud->size(), -1);
  for (size_t i = 0; i < mesh.polygons.size(); ++i) {
    if (mesh.polygons[i].vertices.size() < 3) continue;
    for (size_t k = 0; k < mesh.polygons[i].vertices.size(); ++k) {
      int v = mesh.polygons[i].vertices[k];
      if (vertex_faces[v] < 0 || rand()/(float)RAND_MAX > 1.0/3)
        vertex_faces[v] = i;
    }
  }

  std::map<uint32_t, std::vector<uint32_t> >::iterator it = clusters.begin();
  for (; it != clusters.end(); ++it) {
    const std::vector<uint32_t>& point_indices = it->second;
    if (point_indices.size() <= 3)  continue;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr small_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    small_cloud->resize(point_indices.size());
    for (size_t k = 0; k < point_indices.size(); ++k) 
      small_cloud->points[k]  = cloud->points[point_indices[k]];
      
    if (small_cloud->size() < num_points_per_patch) {
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud (small_cloud);
      srand(time(NULL));
      while (small_cloud->size() < num_points_per_patch) {
        std::vector<int> nn_indices(3,0);
        std::vector<float> nn_dists(3,0);
        int r = rand()%point_indices.size();
        if (vertex_faces[point_indices[r]] > 0) {
          for (int k = 0; k < 3; ++k)
            nn_indices[k] = mesh.polygons[vertex_faces[point_indices[r]]].vertices[k];
        } else {
          tree->nearestKSearch(r, 3, nn_indices, nn_dists);
        }
        float r1 = rand() / (float) RAND_MAX;
        float r2 = rand() / (float) RAND_MAX;
        float r3 = (1-r1-r2);
        pcl::PointXYZ new_point;
        new_point.getVector3fMap() =
          r1*small_cloud->points[nn_indices[0]].getVector3fMap() + 
          r2*small_cloud->points[nn_indices[1]].getVector3fMap() + 
          r3*small_cloud->points[nn_indices[2]].getVector3fMap(); 
        small_cloud->points.push_back(new_point);
        small_cloud->width++;
      }
    }
    while (small_cloud->size() > num_points_per_patch) {
      int r = rand() % small_cloud->size();
      small_cloud->points.erase(small_cloud->points.begin() + r);
      small_cloud->width--;
    }

    int width = (num_points_per_patch)/2, height = width;
    Eigen::MatrixXf mds_points;
    embed3DPatchInPlane(small_cloud, mds_points);
    std::vector<Eigen::MatrixXf> grids_2d;
    // convert2DEmbeddingToCoordinateGrids(small_cloud, mds_points, grids_2d, width, height);
    Eigen::MatrixXf grid_2d; // = grids_2d[0];
    convert2DEmbeddingToGrid(mds_points, grid_2d, width, height);

    // if (it->first == 4) {
    //   pcl::io::savePLYFile("cluster_ex.ply", *small_cloud);
    //   // std::cout << mds_points << "\n";
    //   FILE *f = fopen("cluster_ex.pgm", "w");
    //   std::vector<unsigned char> chars(grid_2d.rows()*grid_2d.cols(), 0);
    //   for (int y = 0; y < grid_2d.rows(); ++y)
    //     for (int x = 0; x < grid_2d.cols(); ++x) {
    //       chars[y*grid_2d.cols() + x] = grid_2d(y, x)*255;
    //   }
    //   fprintf(f, "P5\n%d %d\n255\n", grid_2d.cols(), grid_2d.rows());
    //   fwrite(&chars[0], chars.size(), 1, f);
    //   fclose(f);
    // }

    patches[it->first] = grid_2d; // patch;
  } 
}

void representMeshAsDBEntry(
    std::string patches_folder, std::ofstream& out_patches_filelist,
    pcl::PolygonMesh& mesh, int counter, int num_points_per_patch = 100) {
  
  std::map<uint32_t, Eigen::MatrixXf> patches;
  std::multimap<uint32_t, uint32_t> cluster_adjacency;
  representMeshAsClusterGraph(mesh, patches, cluster_adjacency, num_points_per_patch);

  std::map<uint32_t, Eigen::MatrixXf>::iterator it = patches.begin();
  int i = 0;
  for (; it != patches.end(); ++it) {
    const Eigen::MatrixXf& grid_2d = it->second;

    std::stringstream ss;
    ss << patches_folder << "/patch_" << counter << "_" << i << ".pgm";
    FILE *f = fopen(ss.str().c_str(), "w");
    std::vector<unsigned char> chars(grid_2d.rows()*grid_2d.cols(), 0);
    for (int y = 0; y < grid_2d.rows(); ++y)
      for (int x = 0; x < grid_2d.cols(); ++x) {
        chars[y*grid_2d.cols() + x] = grid_2d(y, x)*255;
    }
    fprintf(f, "P5\n%d %d\n255\n", grid_2d.cols(), grid_2d.rows());
    fwrite(&chars[0], chars.size(), 1, f);
    fclose(f);
    out_patches_filelist << ss.str() << "\n";
    ++i;
  }
}

int main (int argc, char** argv) {
  std::vector<std::pair<std::string, int> > labelling_lines;
  std::vector<std::pair<std::string, std::string> > shaperef_lines;

  std::string in_filename = argv[1];
  std::string patches_folder = argv[2];
  std::string out_filename = argc > 2 ? argv[3] : "";
  
  if (in_filename.find("filelist") != std::string::npos) {
    std::ifstream infile(in_filename);
    std::string filename;
    if (in_filename.find("label") != std::string::npos) {
      int label;
      while (infile >> filename >> label) {
        labelling_lines.push_back(std::make_pair(filename, label));
      }
    } else {
      std::string shape_id;
      while (infile  >> shape_id >> filename) {
        shaperef_lines.push_back(std::make_pair(filename, shape_id));
      }
    }
  } else {
    shaperef_lines.push_back(std::make_pair(in_filename, "test_shape"));
  }

  std::ofstream ofs(out_filename);

  auto it  =  shaperef_lines.begin();
  int counter = 0;
  for (; it != shaperef_lines.end(); ++it) {
    std::cout << "Processing: " << it->first << "\n";
    pcl::PolygonMesh mesh;
    pcl::io::loadMesh(it->first, mesh);
    representMeshAsDBEntry(patches_folder, ofs, mesh, counter, 100);
    ++counter;
  }

  // Save patches and adjacency to caffe format
  
  return 0;
}
