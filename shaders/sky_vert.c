#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec2 aTexCoords;
out vec4 SSAOPos;
out vec3 LightPos;
out vec2 TexCoords;
out vec3 Normal;
uniform mat4 model;
uniform mat4 view;
uniform mat4 centre_view;
uniform mat4 projection;
uniform vec3 car;
void main()
{
    vec4 viewPos = view * vec4(aPos, 1.0);
    vec4 gpos = projection * viewPos;
    SSAOPos = vec4(viewPos.xyz, 0.0); 
    LightPos = aPos + car;
    TexCoords = aTexCoords;
    Normal = vec3(666.0, 666.0, 666.0);
    gl_Position = gpos;
}