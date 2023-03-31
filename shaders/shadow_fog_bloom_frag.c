#version 330 core
//layout (location = 0) 
out vec4 fragcolour;
// layout (location = 1) out vec4 bloomcolour;
in vec2 TexCoords;

uniform sampler2D colours;
uniform sampler2D shadow;
uniform sampler2D g_fog_position;
uniform sampler2D g_albedo;
uniform sampler2D ssao;
uniform vec3 fogcolour;
uniform float f1;
uniform float f2;
uniform float d1;
uniform float d2;
uniform float near;
uniform float far;
uniform vec3 viewPos;
uniform float bloom_limit;

const int shadow_size = 2;

float getFogFactor(float near, float far, float d, float p)
{
    return clamp((pow(d - near , p) / pow(far - near , p)), 0.0, 1.0);
}

void main()
{
    vec4 FragPos = texture(g_fog_position, TexCoords).xyzw;
    vec2 texelSize = 1. / vec2(textureSize(shadow, 0));
    vec3 shadow_result = vec3(0.);
    int shadowcount = 0;
    float amount = 0.0;
    float maxx = 0.;
    vec3 shad = vec3(0.);
    float shadow_scale = 1.0;
    for (int x = -shadow_size; x <= shadow_size; ++x) 
    {
        for (int y = -shadow_size; y <= shadow_size; ++y) 
        {
            vec2 offset = vec2(float(x * shadow_scale), float((y - shadow_size) * shadow_scale)) * texelSize;
            amount = texture(shadow, TexCoords + offset).a;
            shadow_result += vec3(texture(shadow, TexCoords + offset).rgb);
            shadowcount++;
            maxx += amount;
        }
    }
    float shadowing = 1.0 - clamp(20.0 * (maxx / shadowcount), 0.0, 1.0);
    shadow_result = texture(shadow, TexCoords).rgb * shadowing * texture(g_albedo, TexCoords).rgb;
    vec3 colour_result = texture(colours, TexCoords).rgb;
    float brightness = 0.299 * colour_result.r + 0.587 * colour_result.g + 0.114 * colour_result.b;
    vec3 outcolour = vec3(max(shadow_result.r, colour_result.r),max(shadow_result.g, colour_result.g),max(shadow_result.b, colour_result.b));
    // vec3 outcolour = vec3(max(shadow_result.r, brightness),max(shadow_result.g, brightness),max(shadow_result.b, brightness));
    outcolour = outcolour * texture(ssao, TexCoords).r;
    float d = distance(viewPos, FragPos.xyz);
    float distancefog = getFogFactor(near, far * 1.0, d, f1);
    vec4 fogged = mix(
        vec4(outcolour, 1.0),
        vec4(fogcolour, 1.0),
        clamp(distancefog, 0.0, 1.0)
    );
    fragcolour = fogged;
    // float brightness = 0.299 * outcolour.r + 0.587 * outcolour.g + 0.114 * outcolour.b;
    // if(brightness > bloom_limit)
    //     bloomcolour = vec4(outcolour.rgb, 1.0) * 1.0;
    // else
    //     bloomcolour = vec4(0.0, 0.0, 0.0, 1.0);
}
