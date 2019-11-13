#version 120

attribute vec4 vPosition;
attribute vec4 vColor;

uniform mat4 model_view;
uniform mat4 projection;

varying vec4 color;

void main()
{
    color = vColor;
    
    gl_Position = projection * model_view * vPosition / vPosition.w;
}
 