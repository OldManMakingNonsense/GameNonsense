#version 330 core
layout (location = 0) out vec3 g_position;
layout (location = 1) out vec3 g_normal;
layout (location = 2) out vec4 g_albedo;
layout (location = 3) out vec3 g_fog_position;
layout (location = 4) out vec3 g_colour;
in vec2 TexCoords;
in vec3 SSAOPos;
in vec3 LightPos;
in vec3 Normal;
uniform sampler2D texture_diffuse1;
void main()
{    
    g_position = SSAOPos;
    g_fog_position = LightPos;
    g_normal = Normal;
    g_colour = vec3(0.8);
    vec4 texturecolour =  texture(texture_diffuse1, TexCoords);
    g_albedo = vec4(texturecolour.rgb,1.0);
}