#ifndef WINDOW_H
#define WINDOW_H

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <memory>
#include <string>
#include <stdexcept>

#include "timer.h"
#include "imguiController.h"
#include "inputManager.h"
#include "camera.h"
#include "shader.h"
#include "boids.h"
#include "simulationCPU.h"
#include "mode.h"

class Window {
public:
	Window(size_t width, size_t height, std::string title);
	void init();
	void update(float deltaTime);
	void render();
	void destroy();
	void run();
private:
	GLFWwindow* handle;
	size_t width, height;
	std::string title;
	Mode currentMode;
	SimulationState state;
	std::shared_ptr<ImguiController> gui;
	std::shared_ptr<InputManager> input;
	std::shared_ptr<Camera> camera;
	std::shared_ptr<Boids> boids;
	std::shared_ptr<Shader> shEdge, shBoid, shTest;
	unsigned int edgesVAO, boidsVAO, modelVBO;
	float boxSize;

	glm::mat4 createModelMatrix(float3 position, float3 velocity, float scale);
	void initWindow();
	void renderEdges();
	void renderTestBoid();
	void renderBoids();
	void setupBoidsBuffers();
	void bufferModelMatrices(std::vector<glm::mat4> modelMatrices);
	std::vector<glm::mat4> generateModelMatrices();
	static void framebuffer_size_callback(GLFWwindow* window, int width, int
		height);
	static void mouse_callback(GLFWwindow* window, double xpos, double ypos);
};

#endif