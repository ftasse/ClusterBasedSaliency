#include <pcl/common/time.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl_apps/render_views_tesselated_sphere.h>

#include "utils/cmd_parser.h"
// #include "features/normals.h"
#include "io/off_io.h"

#include <pcl/io/png_io.h>
#include <pcl/conversions.h>
 
// ./apps/generate_pcl_views --model=../../../large_datasets/sketches+models/SHREC14LSSTB_TARGET_MODELS/M000001.off \\
// --output_prefix=../../../experiments/depthmaps/ --verbose=1

int main(int argc, char** argv) {
  // Parse cmd flags
  std::map<std::string, std::string> flags;
  parseCmdArgs(argc, argv, &flags);
  bool arg_error = false;

  std::string model_filename, output_prefix;
  bool verbose; int resolution, num_subdivisions;
  float view_angle;
  if (!(getFlag(flags, "model", "",  &model_filename) &&
        getFlag(flags, "output_prefix", "",  &output_prefix))) {
    printf("Error: specify --model and --output_prefix\n");
    arg_error = true;
  }
  getFlag(flags, "resolution", 256,  &resolution);
  getFlag(flags, "num_subdivisions", 0,  &num_subdivisions);
  getFlag(flags, "verbose", false,  &verbose);
  getFlag(flags, "view_angle", 40.0f, &view_angle);
  if (arg_error) return 1;

  // Load the CAD model from a file.
  vtkSmartPointer<vtkPolyData> object;
  if (model_filename.find(".ply") != std::string::npos)
  {
    vtkPLYReader* reader = vtkPLYReader::New ();
    reader->SetFileName (model_filename.c_str());
    reader->Update ();
    object = (reader->GetOutput ());
  }
  else if (model_filename.find(".vtk") != std::string::npos)
  {
    vtkPolyDataReader* reader = vtkPolyDataReader::New ();
    reader->SetFileName (model_filename.c_str());
    reader->Update ();
    object = (reader->GetOutput ());
  }
  else {
    pcl::PolygonMesh mesh;
    if (!pcl::io::loadPolygonFile(model_filename, mesh)) {
      printf("Error: could not load model from %s.\n", model_filename.c_str());
      return 1;
    }
    object = vtkSmartPointer<vtkPolyData>::New();
    pcl::VTKUtils::convertToVTK(mesh, object);
  }

  std::string basename = 
      generate_output_filename(model_filename, "", "", output_prefix);

  
  // Virtual renderer object.
  pcl::apps::RenderViewsTesselatedSphere render_views;
  render_views.addModelFromPolyData(object);
  // Pixel width of the rendering window, it directly affects the snapshot file size.
  render_views.setResolution(resolution);
  // Horizontal FoV of the virtual camera.
  render_views.setViewAngle(view_angle);
  // If true, the resulting clouds of the snapshots will be organized.
  render_views.setGenOrganized(true);
  // How much to subdivide the icosahedron. Increasing this will result in a lot more snapshots.
  render_views.setTesselationLevel(num_subdivisions);
  // If true, the camera will be placed at the vertices of the triangles. If false, at the centers.
  // This will affect the number of snapshots produced (if true, less will be made).
  // True: 42 for level 1, 162 for level 2, 642 for level 3...
  // False: 80 for level 1, 320 for level 2, 1280 for level 3...
  render_views.setUseVertices(true);
  // If true, the entropies (the amount of occlusions) will be computed for each snapshot (optional).
  render_views.setComputeEntropies(true);
 
  if (verbose) printf("Start rendering views from %s.\n", model_filename.c_str());
  render_views.generateViews();
 
  // Object for storing the rendered views.
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> views;
  // Object for storing the poses, as 4x4 transformation matrices.
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
  // Object for storing the entropies (optional).
  std::vector<float> entropies;
  render_views.getViews(views);
  render_views.getPoses(poses);
  render_views.getEntropies(entropies);
  if (verbose) printf("Rendering %d views done.\n", (int)poses.size());

  for (size_t i = 0; i < views.size(); ++i) {
    pcl::PCLImage image;
    pcl::io::PointCloudImageExtractorFromZField<pcl::PointXYZ> pcie;
    pcie.setPaintNaNsWithBlack (false);
    pcie.setScalingMethod(pcie.SCALING_FULL_RANGE);
    // pcie.setScalingMethod(pcie.SCALING_FIXED_FACTOR);
    // pcie.setScalingFactor(254);
    pcie.extract(*views[i], image);
    char output[512] = ""; sprintf(output, "%s_%zu.png", basename.c_str(), i);
    // printf("%d: %d %d %d %d\n", i, image.data[0], image.data[1], *std::min_element(image.data.begin(), image.data.end()), *std::max_element(image.data.begin(), image.data.end()));
    
    // for (size_t j = 0; j < image.data.size(); ++j)
    //   if (image.data[j] == 0)  image.data[j] = 255;
    for (size_t j = 0; j < views[i]->points.size (); ++j)
      if (!pcl::isFinite ((*views[i])[j])) std::memset (&image.data[j * 2], 255, 2);

    pcl::io::savePNGFile (output, image);
  }

  std::ofstream ofs((basename+"_renderinfo.txt").c_str());
  ofs << "#entropy" << "\n";
  for (size_t i = 0; i < poses.size(); ++i) {
    ofs << entropies[i] << " ";
  }
  ofs << "#poses" << "\n";
   for (size_t i = 0; i < poses.size(); ++i) {
    ofs << Eigen::Map<Eigen::VectorXf>(poses[i].data(),poses[i].size()).transpose() << "\n";
  }
  ofs.close();
}