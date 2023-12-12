//#include "Shader.h"
//#include <sstream>
//#include <iostream>
//
//Shader::Shader(const char* vertexPath, const char* fragmentPath)
//{
//	loadShaderSrc(vertexPath, fragmentPath);
//	compileShaders(vertexShaderSrc.c_str(), fragmentShaderSrc.c_str());
//	linkShaders();
//}
//
//Shader::~Shader()
//{
//	glDeleteShader(vertexShader);
//	glDeleteShader(fragmentShader);
//	glDeleteProgram(programID);
//}
//
//void Shader::use()
//{
//	glUseProgram(programID);
//}
//
//void Shader::loadShaderSrc(const char* vertexPath, const char* fragmentPath)
//{
//	std::fstream vertexShaderFile(vertexPath);
//	if (vertexShaderFile.is_open())
//	{
//		std::stringstream ssVertex;
//		ssVertex << vertexShaderFile.rdbuf();
//		vertexShaderSrc = ssVertex.str();
//	}
//	else
//	{
//		std::cout << "Unable to open the file" << std::endl;
//	}
//
//	std::fstream fragmentShaderFile(fragmentPath);
//	if (fragmentShaderFile.is_open())
//	{
//		std::stringstream ssFragment;
//		ssFragment << fragmentShaderFile.rdbuf();
//		fragmentShaderSrc = ssFragment.str();
//	}
//	else
//	{
//		std::cout << "Unable to open the file" << std::endl;
//	}
//}
//
//void Shader::compileShaders(const char* vertexShaderSrc, const char* fragmentShaderSrc)
//{
//	int  success;
//	char infoLog[512];
//
//	// Vertex shader
//	vertexShader = glCreateShader(GL_VERTEX_SHADER);
//	glShaderSource(vertexShader, 1, &vertexShaderSrc, NULL);
//	glCompileShader(vertexShader);
//	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
//	if (!success)
//	{
//		glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
//		std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
//	}
//
//	// Fragment shader
//	fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
//	glShaderSource(fragmentShader, 1, &fragmentShaderSrc, NULL);
//	glCompileShader(fragmentShader);
//	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
//	if (!success)
//	{
//		glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
//		std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
//	}
//}
//
//void Shader::linkShaders()
//{
//	int  success;
//	char infoLog[512];
//
//	programID = glCreateProgram();
//	glAttachShader(programID, vertexShader);
//	glAttachShader(programID, fragmentShader);
//	glLinkProgram(programID);
//
//	// Error checking
//	glGetProgramiv(programID, GL_LINK_STATUS, &success);
//	if (!success) {
//		glGetProgramInfoLog(programID, 512, NULL, infoLog);
//		std::cout << "ERROR::SHADER::PROGRAM::COMPILATION_FAILED\n" << infoLog << std::endl;
//	}
//}
