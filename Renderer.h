#pragma once
#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "Shader.h"
#include "Simulation.h"
#include "Boids.h"

class Renderer
{
private:
	int screenWidth, screenHeight;
	unsigned int posVBO, matVBO, VAO;

public:
	GLFWwindow* window;
	Renderer(int width, int height);
	~Renderer();
	// Initializers
	void initialize();
	void initializeWindow(int screenWidth, int screenHeight);
	void initializeGlad();
	void initializeImgui();
	void initializeBuffers();
	//void setupVertexArrays(Boids& boids);

	glm::mat4 createModelMatrix(float transX, float transY, float angle, float scale);
	void render(Boids& boids, float *baseModelVerts, Shader& sh, Simulation& sim);
	static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
	void processInput(GLFWwindow* window);
};