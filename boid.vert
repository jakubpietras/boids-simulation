#version 330 core

layout (location = 0) in vec3 basePos;

void main()
{
    gl_Position = vec4(basePos.x, basePos.y, basePos.z, 1.0f);
}
