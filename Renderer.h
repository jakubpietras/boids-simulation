#pragma once
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "Shader.h"
#include "Simulation.h"
#include "Boids.h"
#include "GUIController.h"

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
	void initializeBuffers();
	glm::mat4 createModelMatrix(float transX, float transY, float angle, float scale);
	void renderFrame(Boids& boids, float *baseModelVerts, Shader& sh, Simulation& sim, float& deltaTime, GUIController& gui);
	static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
	void processInput(GLFWwindow* window);
};