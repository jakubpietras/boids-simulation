#version 330 core

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;

const vec3 cubeEdges[24] = vec3[24](
    // Bottom face edges
    vec3(0.0f, 0.0f, 0.0f),
    vec3(1.0f, 0.0f, 0.0f),

    vec3(1.0f, 0.0f, 0.0f),
    vec3(1.0f, 1.0f, 0.0f),

    vec3(1.0f, 1.0f, 0.0f),
    vec3(0.0f, 1.0f, 0.0f),

    vec3(0.0f, 1.0f, 0.0f),
    vec3(0.0f, 0.0f, 0.0f),

    // Top face edges (z = 1.0)
    vec3(0.0f, 0.0f, 1.0f),
    vec3(1.0f, 0.0f, 1.0f),

    vec3(1.0f, 0.0f, 1.0f),
    vec3(1.0f, 1.0f, 1.0f),

    vec3(1.0f, 1.0f, 1.0f),
    vec3(0.0f, 1.0f, 1.0f),

    vec3(0.0f, 1.0f, 1.0f),
    vec3(0.0f, 0.0f, 1.0f),

    // Vertical edges
    vec3(0.0f, 0.0f, 0.0f),
    vec3(0.0f, 0.0f, 1.0f),

    vec3(1.0f, 0.0f, 0.0f),
    vec3(1.0f, 0.0f, 1.0f),

    vec3(1.0f, 1.0f, 0.0f),
    vec3(1.0f, 1.0f, 1.0f),

    vec3(0.0f, 1.0f, 0.0f),
    vec3(0.0f, 1.0f, 1.0f)
);

void main()
{
    vec3 pos = cubeEdges[gl_VertexID];
    gl_Position = projection * view * model * vec4(pos, 1.0);
}
