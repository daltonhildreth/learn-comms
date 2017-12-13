#version 330 core
in vec2 ftex;

out vec4 ocolor;

struct Material {
    sampler2D diffuse1;
};

uniform Material material;

void main() {
    ocolor = texture(material.diffuse1, ftex);
}
