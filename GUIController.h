#pragma once
#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"
#include "Simulation.h"

class GUIController
{
private:
	GLFWwindow* window;
	Simulation& sim;
public:
	GUIController(GLFWwindow* window, Simulation& simulation);
	void render(float deltaTime);
	void terminate();
};

