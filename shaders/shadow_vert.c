#version 330 core
layout (location = 0) in vec3 aPos;
// layout (location = 1) in vec3 acolour;

uniform mat4 lightSpaceMatrix;
uniform mat4 projection;
uniform mat4 model;
// out vec3 colour;

void main()
{
	// colour = acolour;
    gl_Position = projection * lightSpaceMatrix * model * vec4(aPos, 1.0);
}  