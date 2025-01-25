#version 330 core

out vec4 FragColor;
in vec3 modelDirection;

void main()
{
    vec3 color = 0.5 + 0.5 * modelDirection; // normalize direction to [0, 1]
    FragColor = vec4(color, 1.0f);
}
