#version 330 core
layout (location = 0) in vec3 ipos;
layout (location = 1) in vec3 inorm;
layout (location = 2) in vec2 itex;

uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;

out vec3 fpos;
out vec3 fnorm;
out vec2 ftex;

void main() {
    vec4 pos = model * vec4(ipos, 1.0);
    fpos = vec3(pos);
    ftex = itex;
    fnorm = inorm;
    gl_Position = proj * view * pos;
}
