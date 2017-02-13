#include <stdio.h>
#include <sstream>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/time.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/search/kdtree.h>

#include "utils/cmd_parser.h"
#include "features/normals.h"
#include "io/off_io.h"
#include "utils/pcl_utils.h"
#include "utils/shading_utils.h"
#include "simplification/salient_based_simplification.h"

struct Quality
{
  float quality;
};
POINT_CLOUD_REGISTER_POINT_STRUCT (
  Quality,           // here we assume a "quality" (as fields)
  (float, quality, quality)
)

int
customSavePLYFileBinary (const std::string &file_name, const pcl::PolygonMesh &mesh)
{
  if (mesh.cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::io::savePLYFile] Input point cloud has no data!\n");
    return (-1);
  }
  // Open file
  std::ofstream fs;
  fs.open (file_name.c_str ());
  if (!fs)
  {
    PCL_ERROR ("[pcl::io::savePLYFile] Error during opening (%s)!\n", file_name.c_str ());
    return (-1);
  }

  // number of points
  size_t nr_points  = mesh.cloud.width * mesh.cloud.height;
  size_t point_size = mesh.cloud.data.size () / nr_points;

  // number of faces
  size_t nr_faces = mesh.polygons.size ();

  // Write header
  fs << "ply";
  fs << "\nformat " << (mesh.cloud.is_bigendian ? "binary_big_endian" : "binary_little_endian") << " 1.0";
  fs << "\ncomment PCL generated";
  // Vertices
  fs << "\nelement vertex "<< mesh.cloud.width * mesh.cloud.height;
  fs << "\nproperty float x"
        "\nproperty float y"
        "\nproperty float z";
 
  // Check if we have intensity or quality 
  int quality_index = pcl::getFieldIndex (mesh.cloud, "quality"),
  intensity_index = pcl::getFieldIndex (mesh.cloud, "intensity"),
  label_index = pcl::getFieldIndex (mesh.cloud, "label");
  if (quality_index != -1)
    fs << "\nproperty float quality";
  if (intensity_index != -1)
    fs << "\nproperty float intensity";
  if (intensity_index != -1)
    fs << "\nproperty float intensity";
  if (label_index != -1)
    fs << "\nproperty int label";

  // Check if we have color on vertices
  int rgba_index = pcl::getFieldIndex (mesh.cloud, "rgba"),
  rgb_index = pcl::getFieldIndex (mesh.cloud, "rgb");
  if (rgba_index != -1)
  {
    fs << "\nproperty uchar red"
          "\nproperty uchar green"
          "\nproperty uchar blue"
          "\nproperty uchar alpha";
  }
  else if (rgb_index != -1)
  {
    fs << "\nproperty uchar red"
          "\nproperty uchar green"
          "\nproperty uchar blue";
  }
  // Faces
  fs << "\nelement face "<< nr_faces;
  fs << "\nproperty list uchar int vertex_index";
  fs << "\nend_header\n";

  // Close the file
  fs.close ();
  // Open file in binary appendable
  std::ofstream fpout (file_name.c_str (), std::ios::app | std::ios::binary);
  if (!fpout)
  {
    PCL_ERROR ("[pcl::io::writePLYFileBinary] Error during reopening (%s)!\n", file_name.c_str ());
    return (-1);
  }

  // Write down vertices
  for (size_t i = 0; i < nr_points; ++i)
  {
    int xyz = 0;
    for (size_t d = 0; d < mesh.cloud.fields.size (); ++d)
    {
      int count = mesh.cloud.fields[d].count;
      if (count == 0)
        count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
      int c = 0;

      // adding vertex
      if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
          mesh.cloud.fields[d].name == "x" ||
          mesh.cloud.fields[d].name == "y" ||
          mesh.cloud.fields[d].name == "z" ))
      {
        float value;
        memcpy (&value, &mesh.cloud.data[i * point_size + mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
        fpout.write (reinterpret_cast<const char*> (&value), sizeof (float));
        // if (++xyz == 3)
        //   break;
        ++xyz;
      } else if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) &&  (
              mesh.cloud.fields[d].name == "quality" || 
                  mesh.cloud.fields[d].name == "intensity"))
      {
        float value;
        memcpy (&value, &mesh.cloud.data[i * point_size + mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
        fpout.write (reinterpret_cast<const char*> (&value), sizeof (float));
      }
      else if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && 
                (mesh.cloud.fields[d].name == "rgb"))

      {
        pcl::RGB color;
        memcpy (&color, &mesh.cloud.data[i * point_size + mesh.cloud.fields[rgb_index].offset + c * sizeof (float)], sizeof (pcl::RGB));
        fpout.write (reinterpret_cast<const char*> (&color.r), sizeof (unsigned char));
        fpout.write (reinterpret_cast<const char*> (&color.g), sizeof (unsigned char));
        fpout.write (reinterpret_cast<const char*> (&color.b), sizeof (unsigned char));
      }
      else if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::UINT32) &&
               (mesh.cloud.fields[d].name == "rgba"))
      {
        pcl::RGB color;
        memcpy (&color, &mesh.cloud.data[i * point_size + mesh.cloud.fields[rgba_index].offset + c * sizeof (uint32_t)], sizeof (pcl::RGB));
        fpout.write (reinterpret_cast<const char*> (&color.r), sizeof (unsigned char));
        fpout.write (reinterpret_cast<const char*> (&color.g), sizeof (unsigned char));
        fpout.write (reinterpret_cast<const char*> (&color.b), sizeof (unsigned char));
        fpout.write (reinterpret_cast<const char*> (&color.a), sizeof (unsigned char));
      }
      else if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::UINT32) &&
               (mesh.cloud.fields[d].name == "rgba"))
      {
        pcl::RGB color;
        memcpy (&color, &mesh.cloud.data[i * point_size + mesh.cloud.fields[rgba_index].offset + c * sizeof (uint32_t)], sizeof (pcl::RGB));
        fpout.write (reinterpret_cast<const char*> (&color.r), sizeof (unsigned char));
        fpout.write (reinterpret_cast<const char*> (&color.g), sizeof (unsigned char));
        fpout.write (reinterpret_cast<const char*> (&color.b), sizeof (unsigned char));
        fpout.write (reinterpret_cast<const char*> (&color.a), sizeof (unsigned char));
      } 
      else if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::UINT32) &&
               (mesh.cloud.fields[d].name == "label"))
      {
        uint32_t  value;
        memcpy (&value, &mesh.cloud.data[i * point_size + mesh.cloud.fields[d].offset + c * sizeof (uint32_t )], sizeof (uint32_t ));
        fpout.write (reinterpret_cast<const char*> (&value), sizeof (uint32_t ));
      }
    }
    if (xyz != 3)
    {
      PCL_ERROR ("[pcl::io::savePLYFile] Input point cloud has no XYZ data!\n");
      return (-2);
    }
  }

  // Write down faces
  for (size_t i = 0; i < nr_faces; i++)
  {
    unsigned char value = static_cast<unsigned char> (mesh.polygons[i].vertices.size ());
    fpout.write (reinterpret_cast<const char*> (&value), sizeof (unsigned char));
    size_t j = 0;
    for (j = 0; j < mesh.polygons[i].vertices.size (); ++j)
    {
      //fs << mesh.polygons[i].vertices[j] << " ";
      int value = mesh.polygons[i].vertices[j];
      fpout.write (reinterpret_cast<const char*> (&value), sizeof (int));
    }
  }

  // Close file
  fs.close ();
}

void saveColoredMesh(
  const pcl::PolygonMesh &my_mesh, 
  const std::vector<float> &scalar_field, 
  const char* filename)
{
  pcl::PolygonMesh mesh = my_mesh;
  std::vector<float> normalized = scalar_field;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
  pcl::PointCloud<Quality>::Ptr q_cloud ( new pcl::PointCloud<Quality> ());
  q_cloud->width = cloud->width; q_cloud->height = cloud->height;
  q_cloud->points.resize(cloud->points.size());
  for (size_t i = 0; i < q_cloud->points.size(); ++i) q_cloud->points[i].quality = normalized[i];
  pcl::PCLPointCloud2 q_cloud2, cloud2; 
  pcl::toPCLPointCloud2(*q_cloud, q_cloud2);
  pcl::toPCLPointCloud2(*cloud, cloud2);
  pcl::concatenateFields (q_cloud2, cloud2, mesh.cloud);

  //std::cout << "Saving to disk ...\n";
  // std::vector< pcl::PCLPointField >  fields = mesh.cloud.fields;
  // for (size_t k = 0; k< fields.size(); ++k) std::cout << fields[k].name << "\n";
  customSavePLYFileBinary(filename, mesh);
  // std::cout << "Saved colored mesh!\n";
}

int main(int argc, char **argv) {
  // Parse cmd flags
  std::map<std::string, std::string> flags;
  parseCmdArgs(argc, argv, &flags);

  std::string model_filename, output_filename;
  if (!getFlag(flags, "model", "",  &model_filename)) {
    printf("Error: specify --model\n");
    return 1;
  }
  if (!getFlag(flags, "output", "",  &output_filename)) {
    printf("Error: specify --output\n");
    return 1;
  }
  int num_simplified_vertices, neighbourhood_size;
  std::string scalar_function_filename;
  std::string save_mesh_with_scalar;
  float neighbourhood_radius_percent;
  getFlag(flags, "num_simplified_vertices", 100, &num_simplified_vertices);
  getFlag(flags, "neighbourhood_size", 0, &neighbourhood_size);
  getFlag(flags, "scalar_function", "",  &scalar_function_filename);
  getFlag(flags, "save_mesh_with_scalar", "",  &save_mesh_with_scalar);
  getFlag(flags, "radius_percent", 0.03f, &neighbourhood_radius_percent);

  // Load model
  pcl::PolygonMesh mesh;
  if (!pcl::io::loadPolygonFile(model_filename, mesh)) {
    printf("Error: could not load model from %s.\n", model_filename.c_str());
    return 1;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr points (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33>);
  pcl::fromPCLPointCloud2(mesh.cloud, *points);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (points);

  float support_radius = neighbourhood_radius_percent/getNormalizingScaleFactor(points, false);
  printf("computed support radius: %f\n", support_radius);

  std::vector<float> scalar_function;
  if (!scalar_function_filename.empty())
      loadScalarFunction(scalar_function_filename.c_str(), scalar_function);

  if (scalar_function.size() != points->size()) {
    std::cout << "Use default saliency. " << scalar_function.size() << "\n";
    scalar_function.clear();
    scalar_function.resize(points->size(), 1.0);
  }

  if (!save_mesh_with_scalar.empty() && scalar_function.size() == points->size()) {
    /*pcl::PointCloud<pcl::Normal>::Ptr scalar_normals (new pcl::PointCloud<pcl::Normal>);
    scalar_normals->resize(points->size());
    for (int i = 0; i < points->size(); ++i) {
      scalar_normals->points[i].normal_x = 
      scalar_normals->points[i].normal_y = 
      scalar_normals->points[i].normal_z = scalar_function[i];
    }
    pcl::PolygonMesh tmp_mesh; tmp_mesh.polygons = mesh.polygons;
    pcl::PointCloud<pcl::RGB>::Ptr colors (new pcl::PointCloud<pcl::RGB>);
    colors->resize(points->size());
    concatenatePointsNormalsRGBs(points, scalar_normals, colors, tmp_mesh.cloud);
    pcl::io::savePolygonFile(save_mesh_with_scalar.c_str(), tmp_mesh);*/
    saveColoredMesh(mesh, scalar_function, save_mesh_with_scalar.c_str());
  }
  
  computeNormals(points, normals, support_radius, tree);
  computeAndOrientNormalsWithCGAL(points, normals, std::max(9,std::min(18, pcl::computeAverageDensity(points, support_radius, tree))), true);
  printf("computed normals.\n");  

  SalientBasedSimplification simplification;
  pcl::PolygonMesh simplified_mesh;
  pcl::StopWatch watch;

  if (num_simplified_vertices == 0) simplified_mesh = mesh;
  else {
    if (mesh.polygons.size() >= 0) {
      simplification.compute(
        mesh, scalar_function, num_simplified_vertices, simplified_mesh);
    } else  {
      pcl::PointCloud<pcl::PointXYZ>::Ptr simplified_points (
        new pcl::PointCloud<pcl::PointXYZ>);
      simplification.compute(
        points, normals, scalar_function, num_simplified_vertices, simplified_points);
      pcl::toPCLPointCloud2(*simplified_points, simplified_mesh.cloud);
    }
  }

  std::cout << "Elapsed simplification time: " << watch.getTimeSeconds() << "\n";
  printf("num of faces: %zd. num of simplified points: %d.\n", 
    simplified_mesh.polygons.size(), num_simplified_vertices);

  pcl::io::savePolygonFile(output_filename.c_str(), simplified_mesh);
  return 0;
}
