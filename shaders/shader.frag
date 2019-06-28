#version 130

in vec4 vertex_coord;
in vec4 vertex_normal;
in vec4 vertex_color;

out vec4 color;

uniform float gap;
uniform vec4 light_normal;
uniform vec4 ambient_light;
uniform vec4 diffuse_light;
uniform vec4 camera_position;

void main(void)
{
    float distance = length(-camera_position - vertex_coord) + 0.00001;
    float attenuation = min(max((2.0-distance/30), 0.4), 1.5);
    vec4 V = normalize(vertex_normal);
    vec4 L = normalize(light_normal);
    vec4 ambient_color = ambient_light;
    vec4 diffuse_color = diffuse_light * max(dot(V, L), 0.0) * attenuation;;
    vec4 coef = vec4(vec3(1.0),1.0);
    color = vertex_color * (diffuse_color + ambient_color);
}
