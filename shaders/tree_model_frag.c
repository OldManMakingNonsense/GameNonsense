#version 330 core
layout (location = 0) out vec4 g_position;
layout (location = 1) out vec3 g_normal;
layout (location = 2) out vec4 g_albedo;
layout (location = 3) out vec4 g_fog_position;
layout (location = 4) out vec3 g_colour;
in vec4 SSAOPos;
in vec3 ourcolour;
flat in vec3 Normal;
in vec3 viewNormal;
in vec2 TexCoords;
flat in vec4 LightPos;
uniform sampler2D texture_diffuse1;
void main()
{    
    g_position = SSAOPos;
    g_fog_position = LightPos;
    g_normal = Normal;
    g_colour = vec3(1.0);
    vec4 texturecolour =  texture(texture_diffuse1, TexCoords) * 1.5;
    g_albedo = vec4(texturecolour.rgb, 1.0);
}