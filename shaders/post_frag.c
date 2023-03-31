#version 330 core
out vec3 FragColor;
in vec2 TexCoords;
uniform sampler2D bloom;
uniform sampler2D light;
const int blur_size = 2;
const float blur_scale = 3.33;
const int shadow_size = 2;
void main()
{
    float maxdist = sqrt(pow(blur_size,2)+pow(blur_size,2));
    vec2 texelSize = 1. / vec2(textureSize(bloom, 0));
    vec3 bloomresult = vec3(0.);
    int bloomcount = 0;
    for (int x = -blur_size; x <= blur_size; ++x) 
    {
        for (int y = -blur_size; y <= blur_size; ++y) 
        {
            vec2 offset = vec2(float(x * blur_scale), float(y * blur_scale)) * texelSize;
            float dist = sqrt(pow(x,2) + pow(y,2));

            bloomresult += (texture(bloom, TexCoords + offset).rgb) * (dist / maxdist);
            bloomcount++;
        }
    }
    vec3 bloomamount = bloomresult / bloomcount;
    vec3 colour = texture(light, TexCoords).rgb;
    float dark = (1.0 - colour.r) * 0.299 + (1.0 - colour.g) * 0.587 + (1.0 - colour.b) * 0.114;
    FragColor = bloomamount + (bloomamount * dark * 0.8) + colour * 1.0;
}
