#version 120

#extension GL_EXT_gpu_shader4 : enable

attribute vec4 vPosition;

uniform mat4 model_view;
uniform mat4 projection;
uniform vec4 agentsData[500];
uniform vec4 agentsColor[100];

varying vec4 color;

const float PI = 3.14159265358979323846264;
const float PI_2 = PI/2.0;
const bool ellipse = false;

float dot2(vec2 a, vec2 b)
{
    return a.x*b.x + a.y*b.y;
}

float cross2(vec2 a, vec2 b)
{
    return a.x*b.y - b.x*a.y;
}

float signedAngle(vec2 a, vec2 b)
{
    return atan(cross2(a, b), dot2(a, b));
}

vec4 rotateY(vec4 v, float angle)
{
    float cosa = cos(angle);
    float sina = sin(angle);
    
    mat4 rotationMatrix = mat4(vec4( cosa, 0, -sina, 0),
                               vec4(    0, 1,    0, 0),
                               vec4( sina, 0, cosa, 0),
                               vec4(    0, 0,    0, 1));
    
    return rotationMatrix * v;
}

void main()
{
    vec4 position = vec4(agentsData[gl_InstanceID].x, 0.0, agentsData[gl_InstanceID].y, 0.0);
    vec2 orientation = agentsData[gl_InstanceID].wz;
    float radius  = 0.5;
    
    vec4 vertex = vec4(radius, 0.0, radius, 1.0) * vPosition;
    
    if(ellipse)
        vertex.x *= 0.5;

    float angle = signedAngle(vec2(1,0),orientation) - PI_2;
        
    if (length(orientation) > 0.0)
        vertex = rotateY(vertex, angle);
    
    vertex += position;
    
    color = agentsColor[gl_InstanceID];
    
    gl_Position = projection * model_view * vertex;
}
 