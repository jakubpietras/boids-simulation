#version 330 core

layout (location = 0) in vec3 basePos;
layout (location = 1) in mat4 modelMatrix;

uniform mat4 projection;

uniform mat4 model;

void main()
{
    gl_Position = projection * model * vec4(basePos, 1.0f);
}
