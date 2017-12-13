#version 330 core
struct Color {
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
};

struct Material {
    //albedo
    Color color;
    //texture
    sampler2D diffuse1;
    sampler2D specular1;
    //properties (e.g. anisotropy/metallicity) Just Specular now.
    float shininess;
};

struct DirLight {
    vec3 dir;

    Color color;
};

struct PointLight {
    vec3 pos;

    //attenuation parameters
    float att_k;
    float att_x;
    float att_x2;

    Color color;
};

//A spotlight is a PointLight with restrictions
struct SpotLight {
    vec3 dir;

    //cutoffs for brightness (100%, 0%)
    float cutoff_100p;
    float cutoff_0p;

    PointLight point_light;
};

in vec3 fpos;
in vec3 fnorm;
in vec2 ftex;

out vec4 ocolor;

uniform mat4 view;
uniform Material material;

//TODO: variable size of lights?
#define LIGHT_LIMIT 8
uniform int n_dir_lights = 0;
uniform DirLight dir_lights[LIGHT_LIMIT];
uniform int n_point_lights = 0;
uniform PointLight point_lights[LIGHT_LIMIT];
uniform int n_spot_lights = 0;
uniform SpotLight spot_lights[LIGHT_LIMIT];

uniform vec3 eye_pos;

Color albedo(Material mtl);
vec3 shade_blinn_phong(vec3 to_L, vec3 norm, vec3 view_dir,
    Color unlit, Color light);
vec3 shade_DirLight(DirLight dl, vec3 norm, vec3 view_dir, Color unlit);
vec3 shade_PointLight(PointLight pl, vec3 norm, vec3 view_dir, Color unlit);
vec3 shade_SpotLight(SpotLight sl, vec3 norm, vec3 view_dir, Color unlit);

void main() {
    vec3 norm = normalize(fnorm);
    vec3 view_dir = normalize(eye_pos - fpos);

    Color unlit = albedo(material);

    vec3 color = vec3(0, 0, 0);
    for (int i = 0; i < n_dir_lights; ++i) {
        color += shade_DirLight(dir_lights[i], norm, view_dir, unlit);
    }
    for (int i = 0; i < n_point_lights; ++i) {
        color += shade_PointLight(point_lights[i], norm, view_dir, unlit);
    }
    for (int i = 0; i < n_spot_lights; ++i) {
        color += shade_SpotLight(spot_lights[i], norm, view_dir, unlit);
    }

    ocolor = vec4(color, 1.0);
}

Color albedo(Material mtl) {
    Color a;
    a.ambient = mtl.color.ambient + vec3(texture(mtl.diffuse1, ftex));
    a.diffuse = mtl.color.diffuse + vec3(texture(mtl.diffuse1, ftex));
    a.specular = mtl.color.specular + vec3(texture(mtl.specular1, ftex));
    return a;
}

vec3 shade_blinn_phong(vec3 to_L, vec3 norm, vec3 view_dir, Color unlit,
        Color light) {
    float diffuse = max(0.0, dot(norm, to_L));
    vec3 half_dir = normalize(view_dir + to_L);
    float specular = max(0.0, pow(max(0.0, dot(norm, half_dir)),
        material.shininess));

    vec3 lit = vec3(0,0,0);
    lit += light.ambient * unlit.ambient;
    lit += light.diffuse * diffuse * unlit.diffuse;
    lit += light.specular * specular * unlit.specular;
    return lit;
}

vec3 shade_DirLight(DirLight dl, vec3 norm, vec3 view_dir, Color unlit) {
    vec3 to_L = normalize(-dl.dir);
    return shade_blinn_phong(to_L, norm, view_dir, unlit, dl.color);
}

vec3 shade_PointLight(PointLight pl, vec3 norm, vec3 view_dir, Color unlit) {
    vec3 to_L = pl.pos - fpos;
    vec3 lit = shade_blinn_phong(normalize(to_L), norm, view_dir, unlit,
        pl.color);

    float l_dist = length(to_L);
    float attenuation = 1.0 /
        (pl.att_k
        + pl.att_x * l_dist
        + pl.att_x2 * l_dist * l_dist);
    return lit * attenuation;
}

vec3 shade_SpotLight(SpotLight sl, vec3 norm, vec3 view_dir, Color unlit) {
    vec3 lit = shade_PointLight(sl.point_light, norm, view_dir, unlit);
    //would prefer not to compute this twice
    vec3 to_L = normalize(sl.point_light.pos - fpos);
    float angle = dot(to_L, normalize(-sl.dir));
    float ramp = sl.cutoff_100p - sl.cutoff_0p;
    float fade = clamp((angle - sl.cutoff_0p)/(ramp), 0.0, 1.0);

    return lit * fade;
}
