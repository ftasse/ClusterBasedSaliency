#include "display/pcl_window.h"

#include <GL/glew.h>
#include <pcl/conversions.h>

#include "display/glew_utils.h"

PCLWindow::PCLWindow(
	pcl::PolygonMesh *mesh,
	int width, int height, 
	int posX, int posY, 
	const char* title) :
	GLWindow(width, height, posX, posY, title)
{
	mesh_ = mesh;
	num_indices_ = 0;

	shader_program_ = 0;
	position_vbo_ = normal_vbo_ = color_vbo_ = 0;
	pos_attr_id_ = normal_attr_id_ = color_attr_id_ = -1;
	transform_uniform_id_  = -1;

	cloud_  = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
		new pcl::PointCloud<pcl::PointXYZRGB> ());
	normals_ = pcl::PointCloud<pcl::Normal>::Ptr(
		new pcl::PointCloud<pcl::Normal> ());

	pcl::fromPCLPointCloud2(mesh->cloud, *cloud_);
	pcl::fromPCLPointCloud2(mesh->cloud, *normals_);

	// glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
}

PCLWindow::~PCLWindow()
{
	if (shader_program_ > 0) glDeleteProgram(shader_program_);

	if (position_vbo_ > 0) glDeleteBuffers(1, &position_vbo_);
	if (normal_vbo_ > 0) glDeleteBuffers(1, &normal_vbo_);
	if (color_vbo_ > 0) glDeleteBuffers(1, &color_vbo_);
	if (index_vbo_ > 0) glDeleteBuffers(1, &index_vbo_);

	if (vao_ > 0) glDeleteVertexArrays(1, &vao_);
}

void PCLWindow::init()
{
	GLWindow::init();

	glGenBuffers(1, &position_vbo_);
	glGenBuffers(1, &normal_vbo_);
	glGenBuffers(1, &color_vbo_);
	glGenBuffers(1, &index_vbo_);

	glGenVertexArrays(1, &vao_);

	initShaders();
	updateGeometry();
}

void PCLWindow::initShaders() {
	std::vector<char*> feedback_varyings (1, (char*) "f_position");
	shader_program_ = create_shader_program("shaders/vs.glsl", "shaders/fs.glsl", feedback_varyings);
	pos_attr_id_ = bindAttributeVariable(shader_program_, "position");
	normal_attr_id_ = bindAttributeVariable(shader_program_, "normal");
	color_attr_id_ = bindAttributeVariable(shader_program_, "color");
	transform_uniform_id_ = bindUniformVariable(shader_program_, "vp_tranform");
	glUseProgram(shader_program_);
}

void PCLWindow::draw()
{
	glUniformMatrix4fv(transform_uniform_id_, 1, GL_FALSE, &camera()->matrix()[0][0]);
	glBindVertexArray(vao_);

	if (index_vbo_ > 0) {
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_vbo_); 
		glDrawElements(
		 	GL_TRIANGLES,      // mode
		 	num_indices_,    // count
		 	GL_UNSIGNED_INT,   // type
		 	(void*)0           // element array buffer offset
		);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	}

	glBindVertexArray(0);
}

void PCLWindow::getTransformFeedback(std::vector<float>& feedback, size_t feedback_size) {
	unsigned int tbo;
	glGenBuffers(1, &tbo);
	glBindBuffer(GL_TRANSFORM_FEEDBACK_BUFFER, tbo);
	glBufferData(GL_TRANSFORM_FEEDBACK_BUFFER,
			     sizeof(float)*feedback_size, NULL, GL_STATIC_READ);
	
	glEnable(GL_RASTERIZER_DISCARD); 
	glUniformMatrix4fv(transform_uniform_id_, 1, GL_FALSE, &camera()->matrix()[0][0]);
	glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, tbo);
	glBindVertexArray(vao_);
	glBeginTransformFeedback(GL_POINTS);
	glDrawArrays(GL_POINTS, 0, cloud_->size());
	glEndTransformFeedback();
	glBindVertexArray(0);
	glDisable(GL_RASTERIZER_DISCARD);
	glFlush();

	feedback.resize(feedback_size, 1);
	glGetBufferSubData(GL_TRANSFORM_FEEDBACK_BUFFER, 0, sizeof(float)*feedback_size, &feedback[0]);
	glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, 0);
	glBindBuffer(GL_TRANSFORM_FEEDBACK_BUFFER, 0);
	glDeleteBuffers(1, &tbo);
}

void PCLWindow::updateVertices() {
	if (mesh_ == NULL) return;

	std::vector<float> vertex_positions;
	std::vector<float> vertex_normals;
	std::vector<float> vertex_colors;

	assert(cloud_->size() == normals_->size());
	vertex_positions.resize(cloud_->size()*3);
	vertex_normals.resize(normals_->size()*3);
	vertex_colors.resize(cloud_->size()*4);

	for (size_t i = 0; i < cloud_->size(); ++i) {
		glm::vec3 position, normal;
		glm::vec4 color;
		get_vertex_info(i, position, normal, color);
		for (int k = 0; k < 3; ++k) vertex_positions[i*3 + k] = position[k];
		for (int k = 0; k < 3; ++k) vertex_normals[i*3 + k] = normal[k];
		for (int k = 0; k < 4; ++k) vertex_colors[i*4 + k] = color[k];
	}

	glBindVertexArray(vao_);
	if (position_vbo_ > 0 && pos_attr_id_ >= 0)
	{
		glBindBuffer(GL_ARRAY_BUFFER, position_vbo_);
		glBufferData(
			GL_ARRAY_BUFFER, 
			sizeof(float)*vertex_positions.size(), 
			&vertex_positions[0], 
			GL_STATIC_DRAW);

  		glEnableVertexAttribArray(pos_attr_id_);
		glVertexAttribPointer(
			pos_attr_id_,		// attribute
			3,                 // number of elements per vertex, here (x,y)
			GL_FLOAT,          // the type of each element
			GL_FALSE,          // take our values as-is
			0,                 // no extra data between each position
			0                  // offset of first element
		);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}
	

	if (color_vbo_ > 0 && color_attr_id_ >= 0)
	{
		glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
		glBindBuffer(GL_ARRAY_BUFFER, normal_vbo_);
		glBufferData(
			GL_ARRAY_BUFFER, 
			sizeof(float)*vertex_colors.size(), 
			&vertex_colors[0], 
			GL_STATIC_DRAW);		

		glEnableVertexAttribArray(color_attr_id_);
		glVertexAttribPointer(
			color_attr_id_, // attribute
			4,                 // number of elements per vertex, here (r,g,b)
			GL_FLOAT,          // the type of each element
			GL_FALSE,          // take our values as-is
			0,                 // no extra data between each position
			0                  // offset of first element
		);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	if (normal_vbo_ > 0 && normal_attr_id_ >= 0)
	{
		glBindBuffer(GL_ARRAY_BUFFER, color_vbo_);
		glBufferData(
			GL_ARRAY_BUFFER, 
			sizeof(float)*vertex_normals.size(), 
			&vertex_normals[0], 
			GL_STATIC_DRAW);
	
		glEnableVertexAttribArray(normal_attr_id_);
		glVertexAttribPointer(
			normal_attr_id_, // attribute
			3,                 // number of elements per vertex, here (r,g,b)
			GL_FLOAT,          // the type of each element
			GL_FALSE,          // take our values as-is
			0,                 // no extra data between each position
			0                  // offset of first element
		);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}
	glBindVertexArray(0);
}

void PCLWindow::updateTopology() {
	if (mesh_ == NULL) {
		num_indices_ = 0;
		return;
	}
	
	std::vector<unsigned int> indices;
	indices.reserve(mesh_->polygons.size()*3);

	for (size_t j = 0; j < mesh_->polygons.size(); ++j)
		for (size_t k = 0; k < mesh_->polygons[j].vertices.size(); ++k)
  			indices.push_back(mesh_->polygons[j].vertices[k]);
  	num_indices_ = indices.size();
	
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_vbo_);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, num_indices_ * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void PCLWindow::updateGeometry()
{
	updateVertices();
	updateTopology();
}

void PCLWindow::get_vertex_info(int i, glm::vec3 &position, glm::vec3 &normal, glm::vec4 &color)
{
	position = glm::vec3 (
		cloud_->points[i].x,
		cloud_->points[i].y,
		cloud_->points[i].z);
	normal = glm::vec3(
		normals_->points[i].normal[0],
		normals_->points[i].normal[1],
		normals_->points[i].normal[2]
		);
	color = glm::vec4(
		cloud_->points[i].r/255.0,
		cloud_->points[i].g/255.0,
		cloud_->points[i].b/255.0,
		1.0
		);
}
