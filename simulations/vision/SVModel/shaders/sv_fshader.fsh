#version 120

uniform vec2 agentVelocity;
uniform vec2 agentPosition;
uniform vec2 goalVelocity;
uniform float agentRadius;
uniform float ttg;

varying float alpha;
varying float alpha_Dot;
varying float depth;
varying float speed_Alpha;
varying float speed_Alpha_Goal;
varying float speed;
varying float PA;

const float PI = 3.14159265358979323846264;
const float PI_2 = PI/2.0;

void main()
{
    // asymetry for agent passing first and second
    const vec3 params[2] = vec3[2](vec3(0.0, 0.6, 1.5), vec3(0.0, 0.7, 1.5));
    const float threshold = PI_2;
    
    vec2 theta_Dot = vec2(0.0);

    float ttc, ttcg;
        
    float dist = max(0.0, depth - PA);
    
    ttc = ttcg = 1e30;
    
    if (speed_Alpha > 0.0)
        ttc   = dist / speed_Alpha;
    
    if (speed_Alpha_Goal > 0.0)
        ttcg  = dist / speed_Alpha_Goal;
    
    // if positive, go first
    float first = alpha * alpha_Dot;
    
    int paramIndex = first < 0.0 ? 1 : 0;
    float v = params[paramIndex].x + params[paramIndex].y / pow(ttc, params[paramIndex].z);
    
    v  = clamp(v, 0.0, threshold);
    
    if (ttc > 1.0 && ttcg >= 10.0)
    {
        ttc = 10.0;
    }
    else if ( (ttc < 10.0) && (ttc >= 0.0) &&
              (abs(alpha_Dot) < v)        &&
              (ttg >= ttc || ttc < 1.0)      )
    {
        theta_Dot = vec2(clamp( (- v + alpha_Dot), -threshold, threshold),
                         clamp( (  v + alpha_Dot), -threshold, threshold));
    }
    else if(speed == 0.0 && (ttcg >= 0.0) && (ttcg < 1.0) )    // not colliding, check if directon to the goal is collision free
    {
        theta_Dot = vec2(100.0, -100.0);
        ttc = 5.0;
    }
    
    gl_FragColor = vec4(theta_Dot.x, theta_Dot.y, ttc, first);
}
