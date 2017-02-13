#ifndef GLMANAGER_H
#define GLMANAGER_H

#include <map>

class GLWindow;

class GLManager
{
public:
	GLManager();
	~GLManager();

	// This should be  a singleton class
	static GLManager& getInstance()
    {
        static GLManager instance; // Guaranteed to be destroyed. 

        // Instantiated on first use.
        return instance;
    }
	
	static void addWindow(GLWindow* window)
	{
		getInstance().addWindow_(window);
	}

    GLManager(GLManager const&);      // Don't Implement
    void operator=(GLManager const&); // Don't implement

private:
	void addWindow_(GLWindow* window);
};

#endif // GLMANAGER_H