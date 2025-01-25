#include "shader.h"

Shader::Shader(std::string vertexShaderPath, std::string fragmentShaderPath) 
{
    std::string vertexSource = loadShaderSource(vertexShaderPath);
    std::string fragmentSource = loadShaderSource(fragmentShaderPath);

    std::vector<GLuint> shaders;
    shaders.push_back(compileShader(GL_VERTEX_SHADER, vertexSource));
    shaders.push_back(compileShader(GL_FRAGMENT_SHADER, fragmentSource));

    createProgram(shaders);
    deleteShaders(shaders);
}

Shader::~Shader() 
{
    glDeleteProgram(program_id);
}

std::string Shader::loadShaderSource(std::string& filename) 
{
    std::ifstream istrm(filename);
    if (!istrm.is_open()) 
    {
        throw std::runtime_error("Failed to open " + filename);
    }
    std::stringstream ss;
    ss << istrm.rdbuf();
    istrm.close();
    return ss.str();
}

GLuint Shader::compileShader(GLenum shaderType, const std::string& shaderSource) 
{
    const char* source = shaderSource.c_str();
    auto shader = glCreateShader(shaderType);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    GLint status;
    std::array<char, 512> infoLog;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
    if (GL_FALSE == status)
    {
        glGetShaderInfoLog(shader, infoLog.size(), nullptr, infoLog.data());
        std::string errorString(infoLog.begin(), infoLog.end());
        throw std::runtime_error("Shader compilation failed. " + errorString);
    }
    return shader;
}

void Shader::createProgram(std::vector<GLuint>& shaders) 
{
    program_id = glCreateProgram();
    for (auto shader : shaders) 
    {
        glAttachShader(program_id, shader);
    }
    glLinkProgram(program_id);

    GLint status;
    std::array<char, 512> infoLog;
    glGetProgramiv(program_id, GL_LINK_STATUS, &status);
    if (GL_FALSE == status) 
    {
        glGetProgramInfoLog(program_id, infoLog.size(), nullptr, infoLog.data());
        std::string errorString(infoLog.begin(), infoLog.end());
        throw std::runtime_error("Shader program linking failed. " + errorString);
    }
}

void Shader::deleteShaders(std::vector<GLuint>& shaders) 
{
    for (auto shader : shaders) 
    {
        glDetachShader(program_id, shader);
        glDeleteShader(shader);
    }
}

void Shader::use() const 
{
    glUseProgram(program_id);
}

void Shader::setInt(const std::string& name, int value) const
{
    auto location = glGetUniformLocation(program_id, name.c_str());
    glUniform1i(location, value);
}

void Shader::setMat4(const std::string& name, const glm::mat4& value) const 
{
    auto location = glGetUniformLocation(program_id, name.c_str());
    glUniformMatrix4fv(location, 1, GL_FALSE, glm::value_ptr(value));
}
void Shader::setVec3(const std::string& name, const glm::vec3& value) const 
{
    auto location = glGetUniformLocation(program_id, name.c_str());
    glUniform3fv(location, 1, glm::value_ptr(value));
}
void Shader::setFloat(const std::string& name, const float value) const 
{
    auto location = glGetUniformLocation(program_id, name.c_str());
    glUniform1f(location, value);
}