#include "inputManager.h"

InputManager::InputManager(GLFWwindow* window, std::shared_ptr<Camera> camera)
    : window(window), camera(camera)
{
    lastMouseX = 0.0f;
    lastMouseY = 0.0f;
    firstMouse = true;
}

void InputManager::processKeyboardInput(float deltaTime)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    {
        glfwSetWindowShouldClose(window, true);
    }
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
    {
        camera->handleKeyboardInput(CameraMovement::FORWARD, deltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
    {
        camera->handleKeyboardInput(CameraMovement::BACKWARD, deltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
    {
        camera->handleKeyboardInput(CameraMovement::LEFT, deltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
    {
        camera->handleKeyboardInput(CameraMovement::RIGHT, deltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
    {
        camera->handleKeyboardInput(CameraMovement::UP, deltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
    {
        camera->handleKeyboardInput(CameraMovement::DOWN, deltaTime);
    }
}

void InputManager::processMouseInput(double xpos, double ypos)
{
    if (ImGui::GetIO().WantCaptureMouse)
    {
        return;
    }

    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
    {
        if (firstMouse)
        {
            lastMouseX = xpos;
            lastMouseY = ypos;
            firstMouse = false;
        }
        float xoffset = xpos - lastMouseX;
        float yoffset = lastMouseY - ypos;
        lastMouseX = xpos;
        lastMouseY = ypos;
        camera->handleMouseMovement(xoffset, yoffset);
    }
    else
    {
        firstMouse = true;
    }
}

void InputManager::setCamera(std::shared_ptr<Camera> newCamera)
{
    camera = newCamera;
}