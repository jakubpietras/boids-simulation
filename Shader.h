#pragma once
#include <glad/glad.h>
#include <fstream>

class Shader
{
public:
	GLuint programID;

	Shader(const char* vertexPath, const char* fragmentPath);
	~Shader();
	void use();
	
private:
	std::string vertexShaderSrc, fragmentShaderSrc;
	GLuint vertexShader, fragmentShader;
	void loadShaderSrc(const char* vertexPath, const char* fragmentPath);
	void compileShaders(const char* vertexShaderSrc, const char* fragmentShaderSrc);
	void linkShaders();

	// setters
};