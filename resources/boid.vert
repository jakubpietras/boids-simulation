#version 330 core

layout (location = 0) in mat4 model;

uniform mat4 projection;
uniform mat4 view;

out vec3 modelDirection;

// tetrahedron representing a boid
const vec3 verts[12] = vec3[](
    vec3(0.225,  0.0,     0.0),    
    vec3(-0.075, 0.0,     0.11547),
    vec3(-0.075, -0.1,   -0.0577), 

    vec3(0.225,  0.0,     0.0),
    vec3(-0.075, -0.1,   -0.0577),
    vec3(-0.075,  0.1,   -0.0577),
    
    vec3(0.225,  0.0,     0.0),
    vec3(-0.075,  0.1,   -0.0577),
    vec3(-0.075,  0.0,    0.11547),
    
    vec3(-0.075,  0.0,    0.11547),
    vec3(-0.075, -0.1,   -0.0577),
    vec3(-0.075,  0.1,   -0.0577)
);

void main()
{
    vec3 forwardDirection = normalize((model * vec4(0.0, 0.0, 1.0, 0.0)).xyz);
    modelDirection = forwardDirection;
    gl_Position = projection * view * model * vec4(verts[gl_VertexID], 1.0f);
}
