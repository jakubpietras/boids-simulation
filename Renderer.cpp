#include "Renderer.h"

#include <cmath>
#include <iostream>
#include <chrono>


Renderer::Renderer(int width, int height)
	: screenWidth(width), screenHeight(height)
{
	initialize();
	initializeWindow(screenWidth, screenHeight);
	initializeGlad();
	glViewport(0, 0, screenWidth, screenHeight);
}

Renderer::~Renderer()
{
	glfwTerminate();
}

void Renderer::initialize()
{
	if (!glfwInit())
	{
		std::cerr << "Failed to initialize GLFW" << std::endl;
		return;
	}
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
}

void Renderer::initializeWindow(int screenWidth, int screenHeight)
{
	window = glfwCreateWindow(screenWidth, screenHeight, "LearnOpenGL", NULL, NULL);
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return;
	}
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
}

void Renderer::initializeGlad()
{
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return;
	}
}

void Renderer::initializeBuffers()
{
	std::size_t vec4Size = sizeof(glm::vec4);
	unsigned int posVBOb, matVBOb, VAOb;

	glGenBuffers(1, &posVBOb);
	glGenBuffers(1, &matVBOb);
	glGenVertexArrays(1, &VAOb);

	glBindVertexArray(VAOb);

	glBindBuffer(GL_ARRAY_BUFFER, posVBOb);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (const void*)0);
	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, matVBOb);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 4 * vec4Size, (void*)0);
	glEnableVertexAttribArray(4);
	glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 4 * vec4Size, (void*)(1 * vec4Size));
	glEnableVertexAttribArray(5);
	glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, 4 * vec4Size, (void*)(2 * vec4Size));
	glEnableVertexAttribArray(6);
	glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, 4 * vec4Size, (void*)(3 * vec4Size));

	glVertexAttribDivisor(1, 1);
	glVertexAttribDivisor(2, 1);
	glVertexAttribDivisor(3, 1);
	glVertexAttribDivisor(4, 1);

	glBindVertexArray(0);

	posVBO = posVBOb;
	matVBO = matVBOb;
	VAO = VAOb;
}

glm::mat4 Renderer::createModelMatrix(float transX, float transY, float angle, float scale)
{
	glm::mat4 translation = glm::translate(glm::mat4(1.0f), glm::vec3(transX, transY, 0.0));
	glm::mat4 rotation = glm::rotate(translation, angle, glm::vec3(0.0f, 0.0f, 1.0f));
	glm::mat4 scaling = glm::scale(rotation, glm::vec3(scale, scale, scale));
	return scaling;
}


void Renderer::renderFrame(Boids& boids, float *baseModelVerts, Shader& sh, Simulation& sim, float& deltaTime, GUIController& gui)
{
	float angle = glm::radians(0.0f);
	glm::mat4 modelMatrix = createModelMatrix(100.0f, 50.0f, glm::radians(90.0f), 10);
	glm::mat4 projection = glm::ortho(0.0f, static_cast<float>(screenWidth), 0.0f, static_cast<float>(screenHeight), 0.0f, 1.0f);

	initializeBuffers();
	glBindBuffer(GL_ARRAY_BUFFER, posVBO);
	glBufferData(GL_ARRAY_BUFFER, 9 * sizeof(float), baseModelVerts, GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, matVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::mat4), &modelMatrix, GL_STATIC_DRAW);


	// Rendering
	auto frameStartTime = std::chrono::steady_clock::now();
	
	// XXX
	processInput(window);
	
	// Setting background color
	glClearColor(0.1f, 0.1f, 0.3f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);

	// Shaders
	sh.use();
	sh.setMat4("projection", projection);

	// Buffers and drawing
	glBindVertexArray(VAO);
	for (int i = 0; i < boids.boidsNumber; i++)
	{
		float angle = glm::radians(90.0f) - atan2f(boids.velocityY[i], boids.velocityX[i]);
		modelMatrix = createModelMatrix(boids.positionX[i], boids.positionY[i], -angle, 10);
		sh.setMat4("model", modelMatrix);
		glDrawArrays(GL_TRIANGLES, 0, 3);
	}

	// ImGUI
	gui.render(deltaTime);

	// glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
	glfwSwapBuffers(window);
	glfwPollEvents();

	auto frameEndTime = std::chrono::steady_clock::now();
	deltaTime = std::chrono::duration<float>(frameEndTime - frameStartTime).count();	
}

void Renderer::framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	glViewport(0, 0, width, height);
}

void Renderer::processInput(GLFWwindow* window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);
}
