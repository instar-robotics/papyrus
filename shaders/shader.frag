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
    if(abs(mod(vertex_coord.x,gap)) < 0.01 || abs(mod(vertex_coord.z,gap)) < 0.01)
    {
	color = vec4(0.0,0.0,0.0,0.0);
    }
    else
    {
	float distance = length(camera_position - vertex_coord) + 0.00001;
	float attenuation = (1-distance/50)*5;
	if(attenuation < 0)
	    attenuation = 0;

	vec4 ambient_color = ambient_light;
	vec4 diffuse_color = diffuse_light * dot(vertex_normal, normalize(light_normal)) * attenuation;

	color = vertex_color * (ambient_color + diffuse_color);
    }
}
