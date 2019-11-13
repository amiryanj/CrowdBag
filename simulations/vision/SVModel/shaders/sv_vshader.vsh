#version 120

#extension GL_EXT_gpu_shader4 : enable
#extension GL_EXT_bindable_uniform: enable

uniform mat4 model_view;
uniform mat4 projection;
uniform vec2 agentVelocity;
uniform vec2 agentPosition;
uniform vec2 goalVelocity;
uniform float agentRadius;
uniform int agentInstanceIndex;
uniform float ttg;

// positions for 201 agents (px,py,vx,vy,ox,oy) + obstacles (0,0,0,0,0,0)
uniform mat3x2 pos_Vertex_Buff[201];// [0].xy - position | [1].xy - velocity | [2].xy - orientation

varying float alpha;
varying float alpha_Dot;
varying float depth;
varying float speed_Alpha;
varying float speed_Alpha_Goal;
varying float speed;
varying float PA;

attribute vec4 vPosition;

const float PI = 3.14159265358979323846264;
const float PI_2 = PI/2.0;

// Set true for drawing cones with elliptical base. You must set true in simulation_vshader.vsh too.
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

float fixAngleValue(float angle)
{
    if (abs(angle) > PI )
        return angle - sign(angle) * 2.0 * PI;
    
    return angle;
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
    vec4 vertex = vec4(0.0, 0.0, 0.0, 1.0);
    
    if (agentInstanceIndex != gl_InstanceID)
    {
        PA = agentRadius;
        
        vec4 mm = vec4(1.0); // for obstacle do nothing
        vec2 lPos_Vertex = pos_Vertex_Buff[gl_InstanceID][0].xy;
        vec2 lVel_Vertex = pos_Vertex_Buff[gl_InstanceID][1].xy;
        vec2 lOri_Vertex = pos_Vertex_Buff[gl_InstanceID][2].xy;

        if(agentInstanceIndex >= 0)
            mm = vec4(PA, 1.7, PA, 1.0);   // for human scale its size. height = 1.7.
        
        // Scale cone vertex.
        vertex = mm * vPosition;
        
        // If cone has elliptical base, it has to be oriented according agent orientation.
        if (ellipse)
        {
            float angle = signedAngle(vec2(1,0), lOri_Vertex) - PI_2;
        
            vertex.x *= 0.5;
            
            if (length(lOri_Vertex) > 0.0)
                vertex = rotateY(vertex, angle);
        }
        
        // Translate cone vertex.
        vertex += vec4(lPos_Vertex.y, 0.0, lPos_Vertex.x, 0.0);
        
        vec2 vel_Composed      = lVel_Vertex - agentVelocity;
        vec2 vel_Composed_Goal = lVel_Vertex - goalVelocity;
        
        vec2 rel_Pos_Vertex = vertex.zx - agentPosition;
        
        // Depth (to compute ttc)
        depth       = length(rel_Pos_Vertex);
        vec2 dir_C  = normalize(rel_Pos_Vertex);
        vec2 dir_CC = -dir_C;
        
        speed = length(lVel_Vertex);
        
        // SpeedAlpha (to compute ttc)
        speed_Alpha      = dot(vel_Composed,      dir_CC);
        speed_Alpha_Goal = dot(vel_Composed_Goal, dir_CC);
        vec2 dir_CN      = normalize(rel_Pos_Vertex + vel_Composed);
        
        vec2 orientationv = normalize(agentVelocity);
        float alpha_C     = signedAngle(orientationv, dir_C);
        float alpha_CN    = signedAngle(orientationv, dir_CN);
        
        // Ensure -PI < alpha < PI
        alpha = fixAngleValue(alpha_C);
        
        // alphaDot = alpha(t+1s) - alpha(t)
        alpha_Dot = alpha_CN - alpha_C;
        
        // Ensure -PI < alpha_Dot < PI
        alpha_Dot = fixAngleValue(alpha_Dot);
        
        // If distance < converging velocity AND alpha_Dot > 90,
        // then alpha_Dot = 180 - alpha_Dot.
        if(depth <= speed_Alpha && abs(alpha_Dot) > PI_2)
        {
            alpha_Dot = sign(alpha_Dot) * PI - alpha_Dot;
        }
    }
    
    gl_Position = projection * model_view * vertex;
}

