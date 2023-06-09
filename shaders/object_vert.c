#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoords;
out vec3 SSAOPos;
out vec3 LightPos;
out vec2 TexCoords;
out vec3 Normal;
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform vec3 car;
void main()
{
	vec4 modelPos = model * vec4(aPos, 1.0);
	vec4 viewModelPos = view * modelPos;
    LightPos = modelPos.xyz;
    SSAOPos = viewModelPos.xyz;
    TexCoords = aTexCoords;
    mat3 normalMatrix = transpose(inverse(mat3(model)));
    Normal = normalize(normalMatrix * aNormal);
    gl_Position = projection * viewModelPos;
}