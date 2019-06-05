#version 130

in vec4 in_vertex; //coordinates of the vertex
in vec4 in_normal; //normal of the vertex
in vec4 in_color; //rgba color of the vertex

out vec4 vertex_coord;
out vec4 vertex_normal;
out vec4 vertex_color;

uniform mat4 matrix_position;

void main(void)
{
    vertex_coord = in_vertex;
    vertex_normal = in_normal;
    vertex_color = in_color;

    gl_Position = matrix_position * in_vertex;
}
