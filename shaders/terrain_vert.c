#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 acolour;
layout (location = 2) in vec3 aNormal;
layout (location = 3) in vec2 aTexCoord1;
layout (location = 4) in vec2 aTexCoord2;
layout (location = 5) in vec2 aTexCoord3;
layout (location = 6) in vec2 aTexCoord4;
layout (location = 7) in vec2 aTexCoord5;
layout (location = 8) in vec2 aTexCoord6;
layout (location = 9) in vec2 aTexCoord7;
layout (location = 10) in vec2 aTexCoord8;
layout(location = 11) in vec3 aTexMix123;
layout(location = 12) in vec3 aTexMix456;
layout(location = 13) in vec3 aTexMix78;
out vec4 SSAOPos;
out vec4 LightPos;
out vec2 TexCoord1;
out vec2 TexCoord2;
out vec2 TexCoord3;
out vec2 TexCoord4;
out vec2 TexCoord5;
out vec2 TexCoord6;
out vec3 Normal;
out vec3 viewNormal;
out vec3 TexMix123;
out vec3 TexMix456;
out vec3 ourcolour;
//out float depth;
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

float LinearizeDepth(float depth) 
{
    float z = depth * 2.0 - 1.0; // back to NDC 
    return (2.0 * 0.05 * 800.0) / (800.0 + 0.05 - z * (800.0 - 0.05));    
}

void main()
{
    vec4 viewPos = view * vec4(aPos, 1.0);
    vec4 gpos = projection * viewPos;
    SSAOPos = vec4(viewPos.xyz, gpos.z); 
    LightPos = vec4(aPos, 1.0);
    TexCoord1 = aTexCoord1;
    TexCoord2 = aTexCoord2;
    TexCoord3 = aTexCoord3;
    TexCoord4 = aTexCoord4;
    TexCoord5 = aTexCoord5;
    TexCoord6 = aTexCoord6;
    TexMix123 = aTexMix123;
    TexMix456 = aTexMix456;
    ourcolour = acolour;
    Normal = mat3(transpose(inverse(model))) * aNormal;
    gl_Position = gpos;
}