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
    vec4 ambient_color = ambient_light;
    vec4 V = normalize(vertex_normal);
    vec4 L = normalize(light_normal);
    vec4 diffuse_color = diffuse_light * max(dot(V, L), 0.0);

    color = vertex_color * (ambient_color + diffuse_color);
}
