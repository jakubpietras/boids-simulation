#include "GUIController.h"

GUIController::GUIController(GLFWwindow* window, Simulation& simulation)
	: window(window),
	sim(simulation)
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
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();

	ImGui::Begin("Boids Simulation controls");
	ImGui::Text("FPS: %.2f", 1.0f / deltaTime);
	ImGui::InputFloat("Separation", &sim.separationFactor, 0.05f);
	ImGui::InputFloat("Alignment", &sim.alignmentFactor, 0.05f);
	ImGui::InputFloat("Cohesion", &sim.cohesionFactor, 0.005f);
	ImGui::SliderFloat("Radius", &sim.visionRadius, 5.0f, 50.0f);
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
