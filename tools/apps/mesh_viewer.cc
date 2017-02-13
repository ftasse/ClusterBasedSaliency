#include <stdio.h>

#include <pcl/search/kdtree.h>

#include "display/pcl_window.h"
// #include "display/silhouette_window.h"
#include "display/gl_manager.h"
#include "io/off_io.h"
#include "utils/cmd_parser.h"
#include "features/normals.h"

int main(int argc, char **argv) {
  std::map<std::string, std::string> flags;
  parseCmdArgs(argc, argv, &flags);
  std::string model_filename;
  if (!getFlag(flags, "model", "",  &model_filename)) {
    printf("Error: specify --model\n");
    return 1;
  }
  
  pcl::PolygonMesh mesh;
  if (!pcl::io::loadPolygonFile(model_filename, mesh)) {
    printf("Error: could not load model from %s.\n", model_filename.c_str());
    return 1;
  }

  if (!pcl::cloudHasNormals(mesh.cloud)){
    printf("Computing normals\n");
    pcl::PointCloud<pcl::PointXYZ>::Ptr points (
      new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (
      new pcl::PointCloud<pcl::Normal>);
    pcl::fromPCLPointCloud2(mesh.cloud, *points);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    computeNormalsIfNecessary(mesh, points, normals, 18, tree, true, true);
  }
  
  GLManager& manager = GLManager::getInstance();
  PCLWindow window(&mesh, 500, 500, 0, 0, model_filename.c_str());
  // window.background_color = glm::vec4(1, 1, 1, 1);
  manager.addWindow(&window);

  // std::vector<float> feedback;
  // window.getTransformFeedback(feedback, 2000*3);
  // for (size_t i = 0; i < feedback.size(); i++) {
  //   std::cout << feedback[i] << " ";
  //   if ((i+1)%3 == 0) std::cout << "| ";
  // }
  // std::cout << "\n";

  window.startMainLoop();
  return 0;
}
