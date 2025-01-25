#include "camera.h"

Camera::Camera(float screenWidth, float screenHeight, float angularSpeed, float sensitivity)
    : angularSpeed(angularSpeed), sensitivity(sensitivity), pitch(0.0f), yaw(180.0f), radius(0.05f)
{ 
    position = glm::vec3(15.0f, 15.0f, -40.0f);
    front = glm::vec3(0.0f, 0.0f, 0.0f);
    updateProjectionMatrix(screenWidth, screenHeight, 45.0f, 0.1f, 100.0f);
    updateFront();
    updateViewMatrix();
}

void Camera::handleKeyboardInput(CameraMovement direction, float deltaTime)
{
    glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);
    float cameraSpeed = 10.0f * deltaTime;

    // rotate (WASD) and zoom in/out (Q/E)
    switch (direction)
    {
    case CameraMovement::FORWARD:
        position += cameraSpeed * front;
        break;
    case CameraMovement::BACKWARD:
        position -= cameraSpeed * front;
        break;
    case CameraMovement::LEFT:
        position -= glm::normalize(glm::cross(front, up)) * cameraSpeed;
        break;
    case CameraMovement::RIGHT:
        position += glm::normalize(glm::cross(front, up)) * cameraSpeed;
        break;
    case CameraMovement::UP:
        position += cameraSpeed * up;
        break;
    case CameraMovement::DOWN:
        position -= cameraSpeed * up;
        break;
    default:
        return;
    }
    updateViewMatrix();
}

void Camera::handleMouseMovement(float xoffset, float yoffset)
{
    xoffset *= sensitivity;
    yoffset *= sensitivity;

    updateRotation(-xoffset, yoffset);
    updateFront();
    updateViewMatrix();
}

void Camera::updateViewMatrix()
{
    glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);
    view = glm::lookAt(position, position + front, up);
}

void Camera::updateFront()
{
    auto rotation = glm::quat(glm::vec3(glm::radians(pitch), glm::radians(yaw), 0.0f));
    front = glm::rotate(rotation, glm::vec3(0.0f, 0.0f, -1.0f));
}

void Camera::updateRotation(float deltaHorizontal, float deltaVertical)
{
    yaw += deltaHorizontal;

    if (glm::abs(yaw) > 360) 
        yaw = 0;
    
    pitch += deltaVertical;
    if (pitch > 89.0f) 
        pitch = 89.0f;
    if (pitch < -89.0f) 
        pitch = -89.0f;
}

void Camera::updateProjectionMatrix(float newWidth, float newHeight, float fov_angle, float nearPlane, float farPlane)
{
    updateFieldOfView(fov_angle);
    updateNearFarPlane(nearPlane, farPlane);
    updateAspectRatio(newWidth, newHeight);
}

void Camera::updateAspectRatio(float newWidth, float newHeight)
{
    aspectRatio = newWidth / newHeight;
    projection = glm::perspective(glm::radians(fov), aspectRatio,
        near, far);
}

void Camera::updateFieldOfView(float angle)
{
    fov = angle;
}

void Camera::updateNearFarPlane(float nearPlane, float farPlane)
{
    near = nearPlane;
    far = farPlane;
}