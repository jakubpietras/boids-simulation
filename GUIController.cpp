#include "GUIController.h"

GUIController::GUIController(GLFWwindow* window, Simulation& simulation, float radiusMax)
	: window(window),
	sim(simulation),
	radiusMax(radiusMax)
{
	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
	//io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;         // IF using Docking Branch

	// Setup Platform/Renderer backends
	ImGui_ImplGlfw_InitForOpenGL(window, true);          // Second param install_callback=true will install GLFW callbacks and chain to existing ones.
	ImGui_ImplOpenGL3_Init("#version 330");
}

void GUIController::render(float deltaTime)
{
	// ImGui initialization
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();

	// Widgets
	ImGui::Begin("Boids Simulation controls");
	ImGui::Text("FPS: %.2f", 1.0f / deltaTime);
	ImGui::SliderFloat("Separation", &sim.separationFactor, 0.05f, 20.0f);
	ImGui::SliderFloat("Alignment", &sim.alignmentFactor, 0.05f, 1.0f);
	ImGui::SliderFloat("Cohesion", &sim.cohesionFactor, 0.005f, 20.0f);
	ImGui::SliderFloat("Radius", &sim.visionRadius, 5.0f, radiusMax);
	ImGui::End();

	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void GUIController::terminate()
{
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
}
