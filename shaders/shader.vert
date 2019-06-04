#version 130

in vec4 in_vertex; //coordinates of the vertex
in vec4 in_normal; //normal of the vertex
in vec4 in_color; //rgba color of the vertex

out vec4 vertex_color;
out vec4 vertex_normal;

uniform mat4 matrix_position;

void main(void)
{
    vertex_color = in_color;
    vertex_normal = in_normal;

    gl_Position = matrix_position * in_vertex;
}
