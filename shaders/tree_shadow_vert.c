#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 3) in mat4 aInstanceMatrix;
uniform mat4 lightSpaceMatrix;
uniform mat4 projection;
uniform mat4 model;
void main()
{
    gl_Position = projection * lightSpaceMatrix * aInstanceMatrix * vec4(aPos, 1.0);
}  