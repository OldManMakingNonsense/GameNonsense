#version 330 core

const int NR_LIGHTS = 1;

struct Light {
    vec3 Position;
    vec3 Color;
    float Linear;
    float Quadratic;
    float Radius;
};

struct SpotLight {
    vec3 position;
    vec3 direction;
    vec3 colour;
    float cutOff;
    float outerCutOff;
};

layout (location = 0) out vec4 fragcolour;
layout (location = 1) out vec4 shadowcolour;

in vec2 TexCoords;

uniform sampler2D g_position;
uniform sampler2D g_normal;
uniform sampler2D g_albedo;
uniform sampler2D ground_colour;
uniform sampler2D g_fog_position;
uniform sampler2D ssao;
uniform sampler2D light_depth;
uniform float f1;
uniform float f2;
uniform float d1;
uniform float d2;
uniform int SSAO;
uniform vec3 lcol;
uniform Light lights[NR_LIGHTS];
uniform vec3 viewPos;
uniform mat4 projectionview;
uniform mat4 projection;
uniform mat4 lightprojection;
uniform mat4 view;
uniform mat4 lightview;
uniform vec3 sunpos;
uniform vec3 suncolour;
uniform vec3 fogcolour;
uniform float near;
uniform float far;
uniform SpotLight rightspot;
uniform SpotLight leftspot;

float LinearizeDepth(float depth) 
{
    float z = depth * 2.0 - 1.0; // back to NDC 
    return (2.0 * 0.05 * 800.0) / (800.0 + 0.05 - z * (800.0 - 0.05));    
}

float LinearizeDepth2(float depth, float near, float far) 
{
    float z = depth * 2.0 - 1.0; // back to NDC 
    return (2.0 * near * far) / (far + near - z * (far - near));    
}

vec4 CalculateLight(vec4 lightspacepos, vec4 fragPosition, vec3 lightPosition, vec3 lightDirection, vec3 lightColour, float innerCutOff, float outerCutOff, float lightDistance) {
    vec4 result = vec4(0.);
    vec3 toLight = normalize(lightPosition - fragPosition.xyz);
    vec3 lightDir = normalize(lightDirection);
    float angle = acos(dot(-toLight, lightDir));
    float dist = distance(lightPosition, fragPosition.xyz);
    float linear = 0.04;
    float quadratic = 0.0002;
    float attenuation =  1.0 / (1.0 + linear * dist + quadratic * dist * dist);
    float epsilon   = innerCutOff - outerCutOff;
    float intensity = clamp((angle - outerCutOff) / epsilon, 0.0, 1.0);
    float shadowdist = 0.0;

    if (dist < 200.0) {


        vec3 projCoords = (lightspacepos.xyz / lightspacepos.w);
        projCoords *= 0.5;
        projCoords += 0.5;
        float depth = texture(light_depth, projCoords.xy).r;
        float closestDepth = LinearizeDepth2(depth, 1.0, 200.0) * 0.005;
        float fragmentDepth = LinearizeDepth2(projCoords.z,1.0, 200.0) * 0.005; 
        // float fragmentDepth = projCoords.z/200.0; 
        // if (closestDepth > fragmentDepth - 0.1)
        // if (closestDepth > fragmentDepth - (0.001 * fragmentDepth)) 
            result = vec4(attenuation * lightColour * intensity, clamp((fragmentDepth - (0.001 * fragmentDepth)) - closestDepth, 0.0, 1.0));
        // else
            // result = vec4(attenuation * lightColour * intensity, 0.0);
    }
    return result;
}


void main()
{      
    vec4 FragPos = texture(g_fog_position, TexCoords).xyzw;
    vec3 SSAOPos = texture(g_position, TexCoords).xyz;
    vec4 lightspacepos = lightprojection * lightview * inverse(view) * vec4(SSAOPos, 1.0);
    vec3 Normal = normalize(texture(g_normal, TexCoords).xyz);
    vec3 L = vec3(0.0, 0.0, 1.0);
    vec3 Colour = texture(g_albedo, vec2(TexCoords.x, TexCoords.y)).rgb;
    vec3 sundir = sunpos;
    float sundiff;
    if (Normal.x > 600.0)
        sundiff = 1.0;
    else
        sundiff = max(dot(Normal, sundir), 0.0);

    vec3 ambient, diffuse;
    diffuse  = suncolour * sundiff * 0.5;
    ambient = suncolour * 0.5;   
    vec3 lighting = diffuse * 0.75 + diffuse * 0.65 + ambient * 0.4;
    vec3 test;
    vec3 lightstotal = vec3(0.0, 0.0, 0.0);

    shadowcolour = CalculateLight(lightspacepos, FragPos, rightspot.position, rightspot.direction, rightspot.colour, rightspot.cutOff, rightspot.outerCutOff, 200.0);

    lighting += lightstotal;
    vec3 cel = vec3(1.0);
    float lightness = 0.299 * lighting.r + 0.587 * lighting.g + 0.114 * lighting.b;
    if (lightness > 0.65)
        cel = vec3(1.0);
    else if (lightness > 0.63)
        cel = vec3(0.85);
    else if (lightness > 0.5)
        cel = vec3(0.7);
    else if (lightness > 0.48)
        cel = vec3(0.55);
    else if (lightness > 0.35)
        cel = vec3(0.4);
    else if (lightness > 0.33)
        cel = vec3(0.25);
    else
        cel = vec3(0.1);
    lighting *= cel * Colour * texture(ground_colour, TexCoords).rgb;
    // lighting = Colour;
    float fragment_depth = texture(g_position, vec2(TexCoords.x, TexCoords.y+0.0)).w / far;
    float line_width = 5.0 - fragment_depth * 2.0;
    vec2 co = vec2(TexCoords.x, TexCoords.y+0.001*line_width);
    float sample_depth = texture(g_position, co).w / far;
    float line = clamp((1.0-clamp(abs(fragment_depth - sample_depth) * 20.0,0.0, 1.0)) / (1.0 - sample_depth), 0.0, 1.0);
    // vec3 viewDir = normalize(viewPos - FragPos.xyz);
    float difference = abs(fragment_depth - sample_depth) / fragment_depth;
    if (difference > 0.35)
        lighting *= vec3(0.2);
    // lightness = 0.299 * lighting.r + 0.587 * lighting.g + 0.114 * lighting.b;
    // fragcolour = vec4(lightness, lightness, lightness, 1.0);
    fragcolour = vec4(lighting, 1.0);
    // fragcolour = vec4(1.0, 0.0, 1.0, 1.0);
    // fragcolour = vec4(Normal.r, Normal.g, Normal.b, 1.0) * 0.5 + 0.5;
    // fragcolour = vec4(texture(ground_colour, TexCoords).rgb, 1.0);

    // float brightness = dot(fragcolour.rgb, vec3(0.2126, 0.7152, 0.0722));
    // if(brightness > 0.8)
    //     bloomcolour = vec4(fragcolour.rgb, 1.0) * 2;
    // else
    //     bloomcolour = vec4(0.0, 0.0, 0.0, 1.0);
}
