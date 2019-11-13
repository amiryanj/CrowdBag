#version 120

#extension GL_EXT_gpu_shader4 : enable

attribute vec4 vPosition;
attribute vec4 vColor;
attribute vec2 textcoord;

varying vec4 color;
varying vec2 st;

uniform mat4 model_view;
uniform mat4 projection;

void main()
{
    gl_Position = projection * model_view * vPosition / vPosition.w;
    color = vColor;
    st = textcoord;
}
