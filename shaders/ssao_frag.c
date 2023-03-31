#version 330 core
out float FragColor;

in vec2 TexCoords;

uniform sampler2D gPosition;
uniform sampler2D gNormal;
uniform sampler2D texNoise;
uniform vec3 samples[64];
uniform mat4 view;
uniform mat4 projection;
uniform vec2 screen_size;

int kernelSize = 64;
float radius = 1.25;
float bias = 0.325;


void main()
{
    vec2 noiseScale = vec2(screen_size.x/8, screen_size.y/8); 
    vec3 fragPos = texture(gPosition, TexCoords).xyz;
    vec3 normal = texture(gNormal, TexCoords).rgb;
    float occlusion = 0.0;
    normal = normalize(normal);
    vec3 randomVec = normalize(texture(texNoise, TexCoords * noiseScale).xyz);
    vec3 tangent = normalize(randomVec - normal * dot(randomVec, normal));
    vec3 bitangent = cross(normal, tangent);
    mat3 TBN = mat3(tangent, bitangent, normal);
    for(int i = 0; i < kernelSize; ++i)
    {
        vec3 samplePos = TBN * samples[i];
        samplePos = fragPos + samplePos * radius; 
        vec4 offset = vec4(samplePos, 1.0);
        offset = projection * offset;
        offset.xyz /= offset.w;
        offset.xyz = offset.xyz * 0.5 + 0.5; 
        float sampleDepth = texture(gPosition, offset.xy).z;
        float rangeCheck = smoothstep(0.0, 1.0, radius / abs(fragPos.z - sampleDepth));
        occlusion += (sampleDepth >= samplePos.z + bias ? 1.0 : 0.0) * rangeCheck;           
    }
    occlusion = clamp(1.0 - (occlusion / kernelSize), 0.0, 1.0);
    FragColor = occlusion;
}