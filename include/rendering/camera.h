#ifndef CAMERA_H
#define CAMERA_H
#define GLM_ENABLE_EXPERIMENTAL

#include <glad/glad.h>
#include <glm/glm.hpp>
#include "glm/ext/matrix_transform.hpp"
#include "glm/ext/matrix_clip_space.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/gtx/quaternion.hpp"
#include <memory>

enum class CameraMovement 
{
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    UP,
    DOWN
};

class Camera 
{
public:
    glm::mat4 view, projection;
    glm::vec3 position, front;

    Camera(float screenWidth, float screenHeight, float angularSpeed, float sensitivity);
    void handleKeyboardInput(CameraMovement direction, float deltaTime);
    void handleMouseMovement(float xoffset, float yoffset);
    void updateViewMatrix();
    void updateFront();
    void updateRotation(float deltaHorizontal, float deltaVertical);

    // perspective manipulation
    void updateProjectionMatrix(float newWidth, float newHeight, float fov_angle, float nearPlane, float farPlane);
    void updateAspectRatio(float newWidth, float newHeight);
    void updateFieldOfView(float angle);
    void updateNearFarPlane(float near, float far);
private:
    float aspectRatio, fov, near, far;
    float pitch,
        yaw,
        radius,
        angularSpeed,
        sensitivity;
};

#endif