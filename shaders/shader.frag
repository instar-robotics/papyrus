#version 130

in vec4 vertex_coord;
in vec4 vertex_normal;
in vec4 vertex_color;

out vec4 color;

uniform vec4 light_normal;
uniform vec4 ambient_light;
uniform vec4 diffuse_light;
uniform vec4 camera_position;

void main(void)
{

    float distance = length(camera_position - vertex_coord) + 0.00001;
    float attenuation = (1-distance/50)*2;
    //float attenuation = 1.75*(1-log(distance+2.0))+1.5;
    if(attenuation < 0)
	attenuation = 0;

    vec4 ambient_color = ambient_light;
    vec4 diffuse_color = diffuse_light * dot(vertex_normal, normalize(light_normal)) * attenuation;

    color = vertex_color * (ambient_color + diffuse_color);
    //color = vertex_color;
}
