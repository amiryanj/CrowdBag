#version 120

varying vec2 st;
varying vec4 color;
uniform sampler2D renderedTexture;
uniform float time;

void main()
{
    gl_FragColor = color * texture2D(renderedTexture, st);
}
 