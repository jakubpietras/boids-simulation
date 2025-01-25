#include "window.h"
#include <imgui.h>
#include <glm/glm.hpp>
#include "simulationGPU.cuh"

Window::Window(size_t width, size_t height, std::string title)
	: width(width), height(height), title(title), handle(nullptr),
	gui(nullptr), input(nullptr), camera(nullptr), currentMode(Mode::CPU),
	edgesVAO(0), boidsVAO(0), modelVBO(0), boxSize(0),
	shBoid(nullptr), shEdge(nullptr), boids(nullptr), shTest(nullptr)
{
}

void Window::init()
{
	initWindow();
	if (!gladLoadGLLoader((GLADloadproc)(glfwGetProcAddress)))
	{
		throw std::runtime_error("Failed to initialize GLAD");
	}
	
	glEnable(GL_DEPTH_TEST);

	glfwSetWindowUserPointer(handle, this);
	glfwSetFramebufferSizeCallback(handle, framebuffer_size_callback);
	glfwSetCursorPosCallback(handle, mouse_callback);

	// data initialization
	// simulation parameters
	Mode currentMode = Mode::GPU;
	float separationFactor = 0.1f,
		alignmentFactor = 0.1f,
		cohesionFactor = 0.1f,
		visionRadius = 2.0f,
		visionAngle = 360.0f,
		boxSize = 30.0f,
		minSpeed = 4.9f,
		maxSpeed = 8.0f;
	size_t totalBoids = 10000;
	bool isPlaying = true,
		resetBoids = false;

	state = SimulationState({currentMode, separationFactor, alignmentFactor, cohesionFactor, visionRadius, visionAngle, maxSpeed, boxSize, isPlaying, resetBoids });
	gui = std::make_shared<ImguiController>(handle);
	camera = std::make_shared<Camera>(width, height, 10.0f, 0.15f);
	input = std::make_shared<InputManager>(handle, camera);
	shEdge = std::make_shared<Shader>("resources/edge.vert", "resources/edge.frag"); 
	shBoid = std::make_shared<Shader>("resources/boid.vert", "resources/boid.frag");
	boids = std::make_shared<Boids>(totalBoids, minSpeed, maxSpeed, boxSize);
	SimulationCPU::getInstance().init(boids, state.boxSize);

	// opengl buffers initialization
	glGenVertexArrays(1, &edgesVAO);
	glGenVertexArrays(1, &boidsVAO);
	glGenBuffers(1, &modelVBO);
	setupBoidsBuffers();
	bufferModelMatrices(generateModelMatrices());

	SimulationGPU::getInstance().init(boids, boxSize, modelVBO);
}

void Window::update(float deltaTime)
{
	input->processKeyboardInput(deltaTime);

	if (state.currentMode == Mode::CPU)
	{
		if (state.resetBoids)
		{
			state.resetBoids = false;
		}
		SimulationCPU::getInstance().run(state, deltaTime);
		bufferModelMatrices(generateModelMatrices());
	}
	else
	{
		// sim gpu
		if (state.resetBoids)
		{
			SimulationGPU::getInstance().reset();
			state.resetBoids = false;
		}
		SimulationGPU::getInstance().run(state, deltaTime);
	}
}

void Window::render()
{
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	
	renderBoids();
	renderEdges();
}

void Window::destroy()
{
	glBindVertexArray(0);
	glDeleteVertexArrays(1, &edgesVAO);
	glDeleteVertexArrays(1, &boidsVAO);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glDeleteBuffers(1, &modelVBO);
}

void Window::run()
{
	init();
	Timer timer = Timer();
	while (!glfwWindowShouldClose(handle))
	{
		timer.update();
		glfwPollEvents();

		update(timer.getDeltaTime());

		gui->startFrame();
		gui->showSimulationControls(state, timer.getFPS(), boids->totalBoids);
		render();
		gui->render();

		glfwSwapBuffers(handle);
	}
	destroy();
}

void Window::initWindow()
{
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	if (!glfwInit())
	{
		throw std::runtime_error("Failed to initialize GLFW");
	}
	handle = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
	if (!handle)
	{
		glfwTerminate();
		throw std::runtime_error("Failed to create GLFW window");
	}
	glfwMakeContextCurrent(handle);
}

void Window::renderEdges()
{
	// scale the box
	auto model = glm::mat4(1.0f);
	model = glm::scale(model, glm::vec3(state.boxSize));

	shEdge->use();
	shEdge->setMat4("view", camera->view);
	shEdge->setMat4("projection", camera->projection);
	shEdge->setMat4("model", model);

	glBindVertexArray(edgesVAO);
	glDrawArrays(GL_LINES, 0, 24);

	glBindVertexArray(0);
	glUseProgram(0);
}

void Window::renderTestBoid()
{
	shTest->use();
	shTest->setMat4("projection", camera->projection);
	shTest->setMat4("view", camera->view);
	float angle = 0.8f * glfwGetTime();
	shTest->setMat4("model", createModelMatrix(make_float3(0.0f, 0.0f, 0.0f), make_float3(1.0f, 0.0f, 0.0f), 3.0f));
	glBindVertexArray(boidsVAO);
	glDrawArrays(GL_TRIANGLES, 0, 12);
}

void Window::renderBoids()
{
	shBoid->use();
	shBoid->setMat4("projection", camera->projection);
	shBoid->setMat4("view", camera->view);
	glBindVertexArray(boidsVAO);
	glDrawArraysInstanced(GL_TRIANGLES, 0, 12, boids->totalBoids);
	
	glUseProgram(0);
	glBindVertexArray(0);
}

void Window::setupBoidsBuffers()
{
	size_t vec4size = sizeof(glm::vec4);
	
	glBindVertexArray(boidsVAO);
	
	glBindBuffer(GL_ARRAY_BUFFER, modelVBO);

	// column 1
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * vec4size, (const void*)0);
	glVertexAttribDivisor(0, 1);
	// column 2
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 4 * vec4size, (const void*)(vec4size));
	glVertexAttribDivisor(1, 1);
	// column 3
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 4 * vec4size, (const void*)(2 * vec4size));
	glVertexAttribDivisor(2, 1);
	// column 4
	glEnableVertexAttribArray(3);
	glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, 4 * vec4size, (const void*)(3 * vec4size));
	glVertexAttribDivisor(3, 1);

	glBindVertexArray(0);
}

void Window::bufferModelMatrices(std::vector<glm::mat4> modelMatrices)
{
	glBindBuffer(GL_ARRAY_BUFFER, modelVBO);
	glBufferData(GL_ARRAY_BUFFER, modelMatrices.size() * sizeof(glm::mat4), modelMatrices.data(), GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

std::vector<glm::mat4> Window::generateModelMatrices()
{
	auto mats = std::vector<glm::mat4>();

	for (int i = 0; i < boids->totalBoids; i++)
	{
		auto model = createModelMatrix(boids->h_position[i], boids->h_velocity[i], 1.0f);
		mats.push_back(model);
	}

	return mats;
}

glm::mat4 Window::createModelMatrix(float3 position, float3 velocity, float scale)
{
	auto model = glm::mat4(1.0f);
	auto vel = glm::vec3(velocity.x, velocity.y, velocity.z);
	auto pos = glm::vec3(position.x, position.y, position.z);

	auto modelOrientation = glm::vec3(1.0f, 0.0f, 0.0f);
	auto velocityNormalized = glm::normalize(vel);
	auto rotationAxis = glm::cross(modelOrientation, velocityNormalized);
	auto rotationAngle = glm::acos(glm::dot(modelOrientation, velocityNormalized));

	model = glm::translate(model, pos);
	model = glm::scale(model, glm::vec3(scale));
	if (rotationAxis != glm::vec3(0.0f))
		model = glm::rotate(model, rotationAngle, glm::normalize(rotationAxis));

	return model;
}

void Window::framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	Window* win = static_cast<Window*>(glfwGetWindowUserPointer(window));
	if (win)
	{
		glViewport(0, 0, width, height);
		win->camera->updateAspectRatio(static_cast<float>(width), static_cast<float>(height));
	}
}

void Window::mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
	Window* win = static_cast<Window*>(glfwGetWindowUserPointer(window));
	if (win)
	{
		win->input->processMouseInput(xpos, ypos);
	}
}
