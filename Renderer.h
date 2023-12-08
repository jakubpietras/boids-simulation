#pragma once
#include <glad/glad.h>
#include <GLFW/glfw3.h>

class Renderer
{
	GLFWwindow* window;

public:
	Renderer(int screenWidth, int screenHeight);
	void initializeWindow(int screenWidth, int screenHeight);
	void gladLoader();
	void render();
	static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
	void processInput(GLFWwindow* window);
};