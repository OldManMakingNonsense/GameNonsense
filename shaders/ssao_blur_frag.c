#version 330 core
out float FragColor;

in vec2 TexCoords;

uniform sampler2D ssaoInput;

const int blur_size = 1;
const float blur_scale = 1.3;

void main() 
{
    vec2 texelSize = 1.0 / vec2(textureSize(ssaoInput, 0));
    float result = 0.0;
    for (int x = -blur_size; x <= blur_size; ++x) 
    {
        for (int y = -blur_size; y <= blur_size; ++y) 
        {
            vec2 offset = vec2(float(x * blur_scale), float(y * blur_scale)) * texelSize;
            result += texture(ssaoInput, TexCoords + offset).r;
        }
    }
    FragColor = result / (pow(blur_size * 2 + 1, 2));
}  