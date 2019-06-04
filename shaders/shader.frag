#version 130

in vec4 vertex_color;
in vec4 vertex_normal;

out vec4 color;

uniform vec4 light_normal;
uniform vec4 ambient_light;
uniform vec4 diffuse_light;

uniform vec4 view_direction;

void main(void)
{
    vec4 ambient_color = ambient_light;
    vec4 diffuse_color = diffuse_light * dot(vertex_normal, normalize(light_normal));

    color = vertex_color * (ambient_color + diffuse_color);
}
