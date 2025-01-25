#ifndef IMGUICONTROLLER_H
#define IMGUICONTROLLER_H
#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include <glm/glm.hpp>
#include "simulationState.h"

class ImguiController 
{
public:
    ImguiController(GLFWwindow* window);
    ~ImguiController();
    void showSimulationControls(SimulationState& state, const float fpsCount, const int boidCount);
    void startFrame();
    void showDemo();
    void render();
};

#endif