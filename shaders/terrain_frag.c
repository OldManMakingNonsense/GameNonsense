#version 330 core
layout (location = 0) out vec4 g_position;
layout (location = 1) out vec3 g_normal;
layout (location = 2) out vec4 g_albedo;
layout (location = 3) out vec4 g_fog_position;
layout (location = 4) out vec3 g_colour;
in vec4 SSAOPos;
in vec3 ourcolour;
in vec3 Normal;
in vec3 viewNormal;
in vec2 TexCoord1;
in vec2 TexCoord2;
in vec2 TexCoord3;
in vec2 TexCoord4;
in vec2 TexCoord5;
in vec2 TexCoord6;
in vec2 TexCoord7;
in vec2 TexCoord8;
in vec3 TexMix123;
in vec3 TexMix456;
in vec4 LightPos;
//in float depth;
uniform sampler2D texture1;
uniform sampler2D texture2;
uniform sampler2D texture3;
uniform sampler2D texture4;
uniform sampler2D texture5;
uniform sampler2D texture6;

float LinearizeDepth(float depth) 
{
    float z = depth * 2.0 - 1.0; // back to NDC 
    return (2.0 * 0.05 * 800.0) / (800.0 + 0.05 - z * (800.0 - 0.05));    
}

void main()
{    
    g_position = SSAOPos;
    g_fog_position = LightPos;
    g_normal = Normal;
    g_colour = ourcolour;
    vec4 texturecolour =  clamp(
                      (texture(texture1, TexCoord1) * TexMix123.x) +
                      (texture(texture2, TexCoord2) * TexMix123.y) +
                      (texture(texture3, TexCoord3) * TexMix123.z) +
                      (texture(texture4, TexCoord4) * TexMix456.x) * 1.3 +
                      (texture(texture5, TexCoord5) * TexMix456.y) +
                      (texture(texture6, TexCoord6) * TexMix456.z), 0.0, 1.0);
    g_albedo = vec4(texturecolour.rgb, 1.0);
}