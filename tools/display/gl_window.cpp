#include "display/gl_window.h"

#include <stdio.h>
#include <iostream>
#include <algorithm>

#include <GL/glew.h>

#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>

#define GLM_FORCE_RADIANS
#include <glm/gtc/matrix_transform.hpp>

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif

GLCamera::GLCamera()
{
	position = glm::vec3(0, 0, 3);
	orientation = glm::mat4(1.0f);
	viewport = glm::ivec4(0, 0, 1, 1);
	zNear = 0.1f; 
	zFar = 100.0f; 
	fov = 60*M_PI/180.0f;
	is_perspective = true;
}

void GLCamera::update_orientation(float vertical_angle, float horizontal_angle) {
    orientation = glm::rotate(orientation, vertical_angle, glm::vec3(1,0,0));
    orientation = glm::rotate(orientation, horizontal_angle, glm::vec3(0,1,0));
}

float GLCamera::aspect_ratio() const
{
	return (1.0* viewport[2]) / viewport[3];
}

glm::mat4 GLCamera::projection_matrix() const {
    if (is_perspective) 
    	return glm::perspective(fov, aspect_ratio(), zNear, zFar);
    else 
    	//return glm::ortho(0.0f, (float) viewport[2], (float) viewport[3], 0.0f, zNear, zFar);
    	return glm::ortho(-aspect_ratio(), aspect_ratio(), 1.0f, -1.0f, zNear, zFar);
}

glm::mat4 GLCamera::view_matrix() const {
    return glm::translate(orientation, -position);
}

glm::mat4 GLCamera::matrix() const {
    return projection_matrix() * view_matrix();
}

glm::vec3 GLCamera::up() const {
    return glm::vec3(glm::inverse(orientation) * glm::vec4(0,1,0,1));
}

glm::vec3 GLCamera::direction() const {
    return glm::vec3(glm::inverse(orientation) * glm::vec4(0,0,-1,1));
}

glm::vec3 GLCamera::right() const {
    return glm::vec3(glm::inverse(orientation) * glm::vec4(1,0,0,1));
}

GLWindow::GLWindow(int width, int height, int posX, int posY, const char* title):
	glfw_window_(NULL), camera_(new GLCamera()), passive_mode_enabled_(false), 
	frame_(0), time_secs_(0), timebase_secs_(0), fps_(0.0), font_renderer_(NULL)

{
  if (!glfwInit()) {
    printf("Failed to initialise glfw!\n");
  }
  
	foreground_color = glm::vec4(51, 51, 51, 255) * (1/255.0f);
	background_color = glm::vec4(180, 180, 180, 255) * (1/255.0f);
	
	glfw_window_ = glfwCreateWindow(width, height, title, NULL, NULL);
    if (!glfw_window_) {
        printf("Failed to create GLFW window\n");
    } else {
      glfwSetWindowPos (glfw_window_, posX, posY);
    	glfwShowWindow(glfw_window_);
    }
}

GLWindow::~GLWindow()
{
	// printf("Destroy window\n");
	delete font_renderer_;
	delete camera_;
	glfwDestroyWindow(glfw_window_);
	// printf("Destroyed\n");
}

bool GLWindow::isValid() const
{
	return glfw_window_;
}

int GLWindow::show() {
	glfwShowWindow(glfw_window_);
}

int GLWindow::hide() {
	glfwHideWindow(glfw_window_);
}

void GLWindow::renderBitmapString(float x, float y, std::string str)
{
	// printf("%s\n", str.c_str());
	if (font_renderer_) font_renderer_->renderText(str.c_str(), x, y);

	/*glMatrixMode( GL_PROJECTION );
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D( 0, width_, 0, height_);

	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();
	glLoadIdentity();

	glRasterPos2f(x,y);
	for (std::string::iterator c = (&str)->begin(); c != (&str)->end(); ++c) 
	    glutBitmapCharacter(font, *c);

	glMatrixMode( GL_PROJECTION );
	glPopMatrix();
	glMatrixMode( GL_MODELVIEW );
	glPopMatrix();*/
}

void GLWindow::init()
{
	glewExperimental = GL_TRUE; 
	GLenum glew_status = glewInit();
	if (glew_status != GLEW_OK) {
		fprintf(stderr, "GLEW Init Error: %s\n", glewGetErrorString(glew_status));
	}

	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_COLOR_MATERIAL);
	glShadeModel (GL_SMOOTH);

	glClearColor(
		background_color[0], 
		background_color[1], 
		background_color[2], 
		background_color[3]);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, &foreground_color[0]);
	
	// font_renderer_ = new FontRenderer ("fonts/FreeSans.ttf");
}

void GLWindow::computeFPS()
{
	frame_++;
	time_secs_ = glfwGetTime();
	if (time_secs_ - timebase_secs_ > 5.0) {
		fps_ = frame_*1.0/(time_secs_-timebase_secs_);
		timebase_secs_ = time_secs_;
		frame_ = 0;

		// printf("%4.2f\n", fps_);
	}
}

template <class Scalar>
bool GLWindow::offscreenRender(
	const GLCamera& camera, 
	Scalar *data, unsigned int renderbuffer_format, unsigned int renderbuffer_attachment,
	unsigned int data_mode, unsigned int data_type, int width, int height) 
{
	GLCamera prev_camera = *camera_;
	*camera_ = camera;
	
	GLuint fbo = 0, depth_stencil_buf = 0, render_buf = 0;
	
	glGenFramebuffers(1, &fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);

	glGenRenderbuffers(1, &render_buf);
	glBindRenderbuffer(GL_RENDERBUFFER, render_buf);
	glRenderbufferStorage(GL_RENDERBUFFER, renderbuffer_format, width, height);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, renderbuffer_attachment, GL_RENDERBUFFER, render_buf);
	if (renderbuffer_attachment != GL_DEPTH_STENCIL_ATTACHMENT) { // for depth testing
		glGenRenderbuffers(1, &depth_stencil_buf);
		glBindRenderbuffer(GL_RENDERBUFFER, depth_stencil_buf);
		glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, width, height);
		glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, depth_stencil_buf);
		glBindRenderbuffer(GL_RENDERBUFFER, 0);
	}
	
	GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER) ;
	if(status != GL_FRAMEBUFFER_COMPLETE) {
	    printf("failed to make complete framebuffer object %x\n", status);
	} else {
		display();
		screenshot(data, data_mode, data_type, width, height);
	}

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glBindRenderbuffer(GL_RENDERBUFFER, 0);
	glDeleteFramebuffers(1,&fbo);
	glDeleteFramebuffers(1,&depth_stencil_buf);
	glDeleteRenderbuffers(1,&render_buf);

	*camera_ = prev_camera;
	return true;
}

template <class Scalar>
void GLWindow::screenshot(Scalar *data, unsigned int data_mode, unsigned int data_type, int width, int height)  {
	if (width <= 0) width = camera()->window_size[0];
	if (height <= 0) height = camera()->window_size[1];
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glReadPixels(0, 0, width, height, data_mode, data_type, data);
	for (int i = 0; i < width; ++i)
		for (int j = 0; j < height/2; ++j) {
			Scalar tmp = data[i + (height - 1 - j)*width];
			data[i + (height - 1 - j)*width] = data[i + j*width];
			data[i + j*width] = tmp;
		}
}

void GLWindow::displayFPS()
{
	computeFPS();
	if (fps_ <= 0.0)	return;

	char s[256];
	sprintf(s,"FPS:%4.2f", fps_);

	// glColor3f(0.0f,1.0f,1.0f);
	renderBitmapString(10,15, s);
}

void GLWindow::applyTransformations()
{
	glMatrixMode(GL_PROJECTION);
	glLoadMatrixf(&camera_->projection_matrix()[0][0]);

	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf(&camera_->view_matrix()[0][0]);
}

void GLWindow::display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

	applyTransformations();
	draw();

	displayFPS();
	swapBuffers();
}

void GLWindow::swapBuffers() {
	glfwSwapBuffers(glfw_window());
}

void GLWindow::startMainLoop() {
	while (!glfwWindowShouldClose(glfw_window())) {
		display();	
		glfwPollEvents();
	}
}

void GLWindow::draw()
{
	GLUquadric *shape = gluNewQuadric();
    gluQuadricDrawStyle(shape, GLU_FILL);
    gluQuadricNormals(shape, GLU_SMOOTH);
    gluSphere(shape, 0.75, 50, 50);
    gluDeleteQuadric(shape);
}

void GLWindow::reshape(int width, int height)
{
	if(height == 0) height = 1;
	width_ = width; height_ = height;
	glViewport(0, 0, width, height);

	camera_->window_size = glm::ivec2(width, height);
	camera_->viewport = glm::ivec4(0, 0, width, height);
}

void GLWindow::keyPressEvent(GLKeyEvent *event)
{
	const float key_speed = 1.0f; // 1 units / second

	double currentTime = glfwGetTime();
	float deltaTime = float(currentTime - time_secs_);

	if (event->action == GLFW_PRESS || event->action == GLFW_REPEAT)
	{
		// std::cout << "Deltatime: " << key_speed*deltaTime << "\n";
		switch (event->key) {
	        case GLFW_KEY_ESCAPE:
	            glfwSetWindowShouldClose(glfw_window_, GL_TRUE);
	            break;

	        case GLFW_KEY_UP: // Move forward
	        	camera_->position += camera_->direction() * deltaTime * key_speed;
	        	break;

	        case GLFW_KEY_DOWN: // Move backward
	            camera_->position -= camera_->direction() * deltaTime * key_speed;
	            break;

	        case GLFW_KEY_RIGHT: // Strafe right
	            camera_->position  += camera_->right() * deltaTime * key_speed;
	            break;

	        case GLFW_KEY_LEFT: // Strafe left
	            camera_->position -= camera_->right() * deltaTime * key_speed;
	            break;

	        default:
	        	break;
        }
	}
}

void GLWindow::mouseReleaseEvent(GLMouseEvent *event)
{}

void GLWindow::mousePressEvent(GLMouseEvent *event)
{}

void GLWindow::cursorMoveEvent(double xpos, double ypos)
{
	float mouse_speed = 0.01f;

	float xoffset = xpos - latest_mouse_press_event_.xpos;
	float yoffset = ypos - latest_mouse_press_event_.ypos;

	if (latest_mouse_press_event_.action == GLFW_PRESS)
	{
		switch (latest_mouse_press_event_.button) {
	        case GLFW_MOUSE_BUTTON_LEFT:
	            camera_->position  += 
	            	(-camera_->right() * xoffset + camera_->up() * yoffset) * mouse_speed;
	            break;

	        case GLFW_MOUSE_BUTTON_RIGHT:
	            camera_->update_orientation(xoffset*mouse_speed, yoffset*mouse_speed);
	            break;
	    }
	}
}

void GLWindow::scrollEvent(double xoffset, double yoffset)
{
	float scroll_speed = 0.1f;
	camera_->fov = camera_->fov + scroll_speed * std::max(xoffset, yoffset);
}


// Template explicit instantiations
template  bool
GLWindow::offscreenRender<float>(const GLCamera&, float*, unsigned int, unsigned int, unsigned int, unsigned int, int, int);

template  bool 
GLWindow::offscreenRender<unsigned char>(const GLCamera&, unsigned char*, unsigned int, unsigned int, unsigned int, unsigned int, int, int);

template  bool
GLWindow::offscreenRender<unsigned short>(const GLCamera&, unsigned short*, unsigned int, unsigned int, unsigned int, unsigned int, int, int);

template void 
GLWindow::screenshot(float*, unsigned int, unsigned int, int, int);

template void 
GLWindow::screenshot(unsigned char*, unsigned int, unsigned int, int, int);

template void 
GLWindow::screenshot(unsigned short*, unsigned int, unsigned int, int, int);