#ifndef SHADER_H
#define SHADER_H
#include <glad/glad.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <array>
#include <glm/glm.hpp>
#include "glm/gtc/type_ptr.hpp"

class Shader 
{
public:
    Shader(std::string vertexShaderPath, std::string fragmentShaderPath);
    ~Shader();
    void use() const;
    void setInt(const std::string& name, int value) const;
    void setMat4(const std::string& name, const glm::mat4& value) const;
    void setVec3(const std::string& name, const glm::vec3& value) const;
    void setFloat(const std::string& name, const float value) const;

private:
    GLuint program_id;
    std::string loadShaderSource(std::string& shaderPath);
    GLuint compileShader(GLenum shaderType, const std::string& shaderSource);
    void createProgram(std::vector<GLuint>& shaders);
    void deleteShaders(std::vector<GLuint>& shaders);
};

#endif //SHADER_H