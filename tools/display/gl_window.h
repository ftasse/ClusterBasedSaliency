#ifndef GLWINDOW_H
#define GLWINDOW_H

#include <string>
#include <glm/glm.hpp>
#include "font_renderer.h"

class GLFWwindow;

struct GLKeyEvent
{
	GLKeyEvent(int key, int scancode, int action, int mods)
	{
		this->key = key;
		this->scancode = scancode;
		this->action = action;
		this->mods = mods;
	}

 	int key, scancode, action, mods;
};

struct GLMouseEvent
{
	GLMouseEvent(
		int button = -1, int action = -1, int mods = -1, float xpos = 0, float ypos = 0)
	{
		this->button = button;
		this->action = action;
		this->mods = mods;

		this->xpos = xpos;
		this->ypos = ypos;
	}

	int button, action, mods;
	float xpos, ypos;
};

struct GLCamera
{
	glm::vec3 position;
	glm::mat4 orientation;
	glm::ivec4 viewport;
	glm::ivec2 window_size;
	float zNear, zFar, fov;
	bool is_perspective;

	GLCamera();

	void update_orientation(float vertical_angle, float horizontal_angle);

	float aspect_ratio() const;
	glm::mat4 viewport_transform() const;

	glm::mat4 projection_matrix() const;
	glm::mat4 view_matrix() const;
	glm::mat4 matrix() const;

	glm::vec3 up() const;
	glm::vec3 direction() const;
	glm::vec3 right() const;
};

class GLWindow {
public:
	GLWindow(int width, int height, int posX, int posY, const char* title);
	virtual ~GLWindow();

	int window_id() const
	{
		return window_id_;
	}

	void window_id(int value)
	{
		window_id_ = value;
	}

	bool is_passive_mode_enabled() const
	{
		return passive_mode_enabled_;
	}

	void toggle_passive_mode(bool status)
	{
		passive_mode_enabled_ = status;
	}

	GLMouseEvent latest_mouse_press_event() const
	{
		return latest_mouse_press_event_;
	}

	GLFWwindow* glfw_window() const
	{
		return glfw_window_;
	}

	GLCamera* camera() const
	{
		return camera_;
	}

	void latest_mouse_press_event(GLMouseEvent event)
	{
		latest_mouse_press_event_ = event;
	}

	bool isValid() const;

	void computeFPS();
	void displayFPS();

	int show();
	int hide();
	void swapBuffers();

	virtual void renderBitmapString(float x, float y, std::string str);

	virtual void applyTransformations();

	template <class Scalar>
	bool offscreenRender(const GLCamera& camera, 
		Scalar *data, unsigned int renderbuffer_format, unsigned int renderbuffer_attachment, 
		unsigned int data_mode, unsigned int data_type, int width, int height);

	template <class Scalar>
	void screenshot(Scalar *data, unsigned int data_mode, unsigned int data_type, int width = 0, int height = 0);

	virtual void init();
	virtual void draw();
	virtual void reshape(int weight, int height);

	virtual void display();
	virtual void startMainLoop();

	virtual void keyPressEvent(GLKeyEvent *event);
	virtual void mousePressEvent(GLMouseEvent *event);
	virtual void mouseReleaseEvent(GLMouseEvent *event);
	virtual void cursorMoveEvent(double xpos, double ypos);
	virtual void scrollEvent(double xoffset, double yoffset);

	glm::vec4 foreground_color, background_color;
private:
	GLFWwindow* glfw_window_;
	GLCamera* camera_;
	FontRenderer* font_renderer_;

	int window_id_;
	int width_, height_;

	GLMouseEvent latest_mouse_press_event_;
	bool passive_mode_enabled_;

	float time_secs_, timebase_secs_;
	int frame_;
	float fps_;
};

#endif // GLWINDOW_H
