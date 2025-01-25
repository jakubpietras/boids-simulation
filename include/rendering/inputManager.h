#ifndef INPUT_MANAGER_H
#define INPUT_MANAGER_H

#include "camera.h"
#include "imgui.h"
#include <GLFW/glfw3.h>

class InputManager 
{
public:
	InputManager(GLFWwindow* window, std::shared_ptr<Camera> camera);
	void processKeyboardInput(float deltaTime);
	void processMouseInput(double xpos, double ypos);
	void setCamera(std::shared_ptr<Camera> newCamera);
private:
	GLFWwindow* window;
	std::shared_ptr<Camera> camera;
	double lastMouseX, lastMouseY;
	bool firstMouse;
};
#endif