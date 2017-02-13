#ifndef PCL_WINDOW_H
#define PCL_WINDOW_H

#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "display/gl_window.h"

class PCLWindow : public GLWindow
{
public:
	PCLWindow(pcl::PolygonMesh *mesh,
			  int width, int height, 
			  int posX, int posY, 
			  const char* title);

	virtual ~PCLWindow();

	virtual void init();
	
	virtual void draw();

	virtual void initShaders();

	virtual void getTransformFeedback(std::vector<float>& feedback, size_t feedback_size);

protected:
	virtual void updateGeometry();
	virtual void updateTopology();
	virtual void updateVertices();

	virtual void get_vertex_info(int i, glm::vec3 &position, glm::vec3 &normal, glm::vec4 &color);

	pcl::PolygonMesh *mesh_;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
	pcl::PointCloud<pcl::Normal>::Ptr normals_;

	unsigned int shader_program_;
	int pos_attr_id_;
	int normal_attr_id_;
	int color_attr_id_;
	int transform_uniform_id_;

	unsigned int vao_, position_vbo_, normal_vbo_, color_vbo_, index_vbo_;
	unsigned int num_indices_;
};

#endif // PCL_WINDOW_H