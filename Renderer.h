#pragma once
#include <glad/glad.h>
#include <GLFW/glfw3.h>

class Renderer
{
private:
	GLFWwindow* window;
	int screenWidth;
	int screenHeight;
	GLuint shaderProgram;
	GLuint VBO_X, VBO_Y;
	GLuint VAO;
	const char* vertexShader = "\0";

public:
	Renderer(int width, int height);
	~Renderer();
	// Initializers
	void initialize();
	void initializeWindow(int screenWidth, int screenHeight);
	void initializeGlad();

	// Shader methods


	void render();
	static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
	void processInput(GLFWwindow* window);
};