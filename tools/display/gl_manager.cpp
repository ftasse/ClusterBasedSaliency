#include "display/gl_manager.h"
#include "display/gl_window.h"

#include <GLFW/glfw3.h>

#include <stdlib.h>
#include <stdio.h>

GLWindow* getWindow(GLFWwindow* glfw_window)
{
	if (glfw_window == 0)
		return NULL;
	else
		return (GLWindow *) glfwGetWindowUserPointer(glfw_window);
}

GLWindow* getCurrentWindow()
{
	GLFWwindow* glfw_window = glfwGetCurrentContext();
	return getWindow(glfw_window);
}

static void reshapeCallback(
	GLFWwindow *glfw_window, int width, int height)
{
	getWindow(glfw_window)->reshape(width, height);
}

static void keyCallback(
	GLFWwindow *glfw_window, int key, int scancode, int action, int mods)
{
	GLKeyEvent event(key, scancode, action, mods);
	getWindow(glfw_window)->keyPressEvent(&event);
}

static void mouseButtonCallback(
	GLFWwindow *glfw_window, int button, int action, int mods)
{
	double xpos, ypos;
	glfwGetCursorPos(glfw_window, &xpos, &ypos);

	GLMouseEvent event(button, action, mods, xpos, ypos);
	getWindow(glfw_window)->latest_mouse_press_event(event);

	if (action == GLFW_PRESS)
		getWindow(glfw_window)->mousePressEvent(&event);
	else
		getWindow(glfw_window)->mouseReleaseEvent(&event);
}

static void cursorPosCallback(
	GLFWwindow *glfw_window, double xpos, double ypos)
{
	GLWindow *window = getWindow(glfw_window);
	if (window->latest_mouse_press_event().action != GLFW_PRESS && 
		!window->is_passive_mode_enabled())
		return;

	window->cursorMoveEvent(xpos, ypos);
	GLMouseEvent event = window->latest_mouse_press_event();
	event.xpos = xpos;
	event.ypos = ypos;
	window->latest_mouse_press_event(event);
}

static void scrollCallback(
	GLFWwindow *glfw_window, double xoffset, double yoffset)
{
	getWindow(glfw_window)->scrollEvent(xoffset, yoffset);
}

static void errorCallback(int error, const char* description)
{
  // fputs("Display error:\n", stderr);
  fputs(description, stderr);
  fputs("\n", stderr);
}


void windowCloseCallback(GLFWwindow* window)
{
    // if (!time_to_close)
    //    glfwSetWindowShouldClose(window, GL_FALSE);
}

GLManager::GLManager()
{
	glfwSetErrorCallback(errorCallback);
	if (!glfwInit())
        printf("Failed to initialise glfw!\n");
    else
    {
    	glfwDefaultWindowHints();
    	glfwWindowHint(GLFW_VISIBLE, false);
			glfwWindowHint(GLFW_RESIZABLE, true);
			glfwWindowHint(GLFW_DEPTH_BITS, 24);
			glfwWindowHint(GLFW_STENCIL_BITS, 8);
		
			glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
 			glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
			glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
			glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    }
}

GLManager::~GLManager()
{
	glfwTerminate();
}

void GLManager::addWindow_(GLWindow* window)
{
  // glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	if (window->isValid()) {
		GLFWwindow *glfw_window = window->glfw_window();
		glfwSetWindowUserPointer(glfw_window, (void *) window);
		glfwMakeContextCurrent(glfw_window);
		
		printf("OpenGL version: %s\n", glGetString(GL_VERSION));
    
		glfwSetFramebufferSizeCallback(glfw_window, reshapeCallback);
		//glfwSetWindowSizeCallback(glfw_window, reshapeCallback);
		glfwSetKeyCallback(glfw_window, keyCallback);
		glfwSetMouseButtonCallback(glfw_window, mouseButtonCallback);
		glfwSetScrollCallback(glfw_window, scrollCallback);
		glfwSetCursorPosCallback(glfw_window, cursorPosCallback);
		glfwSetWindowCloseCallback(glfw_window, windowCloseCallback);

		int width, height;
		//glfwGetFramebufferSize(glfw_window, &width, &height)
		glfwGetWindowSize(glfw_window, &width, &height);
		reshapeCallback(glfw_window, width, height);

		window->init();
	}
}
