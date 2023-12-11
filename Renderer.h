#pragma once
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "Shader.h"

class Renderer
{
private:
	GLFWwindow* window;
	int screenWidth, screenHeight;
	GLuint VBO_X, VBO_Y, VAO;
	Shader shader;

public:
	Renderer(int width, int height, Shader sh);
	~Renderer();
	// Initializers
	void initialize();
	void initializeWindow(int screenWidth, int screenHeight);
	void initializeGlad();
	void setupVertexArrays();

	void render();
	static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
	void processInput(GLFWwindow* window);
};