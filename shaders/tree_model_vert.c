#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoords;
layout (location = 3) in mat4 aInstanceMatrix;

uniform mat4 view;
uniform mat4 projection;

out vec4 SSAOPos;
flat out vec4 LightPos;
out vec2 TexCoords;
flat out vec3 Normal;
out vec3 ourcolour;

void main()
{
    vec4 viewPos = view * aInstanceMatrix * vec4(aPos, 1.0);
    vec4 gpos = projection * viewPos;
    SSAOPos = vec4(viewPos.xyz, gpos.z); 
    LightPos = aInstanceMatrix * vec4(aPos, 1.0);		
    TexCoords = aTexCoords;
    Normal = mat3(transpose(inverse(aInstanceMatrix))) * aNormal;
    gl_Position = gpos;
}  