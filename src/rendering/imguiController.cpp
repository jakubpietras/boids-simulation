#include "imguiController.h"
#include <string>

ImguiController::ImguiController(GLFWwindow* window) 
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init();
}

ImguiController::~ImguiController() 
{
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

void ImguiController::showSimulationControls(SimulationState& state, const float fpsCount, const int boidCount)
{
    // Introduction
    ImGui::Begin("Boids simulation");
    ImGui::TextWrapped("Welcome to 'Boids simulation'!");
    ImGui::SeparatorText("Navigation");
    ImGui::TextWrapped("* W/S/A/D to move forward/backward/left/right");
    ImGui::TextWrapped("* SPACE to move up");
    ImGui::TextWrapped("* LEFT SHIFT to move down");
    ImGui::TextWrapped("* LEFT MOUSE to move camera");
    // Statistics section
    ImGui::SeparatorText("Stats");
    auto fpsText = "FPS: " + std::to_string(fpsCount);
    ImGui::Text(fpsText.c_str());
    auto boidCountText = "Boid count: " + std::to_string(boidCount);
    ImGui::Text(boidCountText.c_str());
    // Controls section
    ImGui::SeparatorText("Controls");
    if (state.currentMode == Mode::CPU)
    {
        if (ImGui::Button("Use GPU"))
        {
            state.currentMode = Mode::GPU;
            state.resetBoids = true;
        }
        ImGui::SameLine();
        ImGui::BeginDisabled();
        if (ImGui::Button("Use CPU"))
            state.currentMode = Mode::CPU;
        ImGui::EndDisabled();
    }
    else
    {
        ImGui::BeginDisabled();
        if (ImGui::Button("Use GPU"))
            state.currentMode = Mode::GPU;
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("Use CPU"))
        {
            state.currentMode = Mode::CPU;
            state.resetBoids = true;
        }
    }
    ImGui::SameLine();
    std::string playPause = state.isPlaying ? "Pause" : "Play";
    if (ImGui::Button(playPause.c_str()))
    {
        state.isPlaying = !state.isPlaying;
    }
    
    ImGui::SeparatorText("Simulation coefficients");
    ImGui::InputFloat("Separation", &state.separationFactor, 0.001f, 2.0f);
    ImGui::InputFloat("Alignment", &state.alignmentFactor, 0.001f, 2.0f);
    ImGui::InputFloat("Cohesion", &state.cohesionFactor, 0.001f, 2.0f);
    ImGui::InputFloat("Radius", &state.visionRadius, 0.25f, state.boxSize);
    ImGui::InputFloat("Max speed", &state.maxSpeed, 0.5f, 20.0f);
    ImGui::End();
}

void ImguiController::startFrame() 
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void ImguiController::showDemo() 
{
    ImGui::ShowDemoWindow();
}

void ImguiController::render() 
{
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}