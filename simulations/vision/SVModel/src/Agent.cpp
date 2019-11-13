/**
 *  @file Agent.cpp
 *  @author Teofilo Dutra
 */

#include "Agent.h"

Agent::Agent() : m_position(vec3()),
                 m_previousPosition(vec3()),
                 m_velocity(vec3()),
                 m_previousVelocity(vec3()),
                 m_comfortSpeed(0.0f),
                 m_radius(0.0f),
                 m_height(0.0f)
{
}

Agent::Agent(const Agent &agent)
{
    m_position = agent.getPosition();
    m_previousPosition = agent.getPreviousPosition();
    m_velocity = agent.getVelocity();
    m_previousPosition = agent.getPreviousVelocity();
    m_comfortSpeed = agent.getComfortSpeed();
    m_radius = agent.getRadius();
    m_height = agent.getHeight();
}

Agent *Agent::clone()
{
    return new Agent(*this);
}

const vec3& Agent::getPosition() const
{
    return m_position;
}

void Agent::setPosition(const vec3 &position)
{
    m_previousPosition = m_position;
    m_position = position;
}

const vec3& Agent::getPreviousPosition() const
{
    return m_previousPosition;
}

void Agent::setPreviousPosition(const vec3 &previousPosition)
{
    m_previousPosition = previousPosition;
}

const vec3& Agent::getVelocity() const
{
    return m_velocity;
}

void Agent::setVelocity(const vec3 &velocity)
{
    m_previousVelocity = m_velocity;
    m_velocity = velocity;
}

const vec3& Agent::getPreviousVelocity() const
{
    return m_previousVelocity;
}

void Agent::setPreviousVelocity(const vec3 &previousVelocity)
{
    m_previousVelocity = previousVelocity;
}

mFloat Agent::getComfortSpeed() const
{
    return m_comfortSpeed;
}

void Agent::setComfortSpeed(mFloat comfortSpeed)
{
    m_comfortSpeed = comfortSpeed;
}

mFloat Agent::getRadius() const
{
    return m_radius;
}

void Agent::setRadius(mFloat radius)
{
    m_radius = radius;
}

mFloat Agent::getHeight() const
{
    return m_height;
}

void Agent::setHeight(mFloat height)
{
    m_height = height;
}

const vec3& Agent::getOrientation() const
{
    return m_orientation;
}

void Agent::setOrientation(const vec3 &orientation)
{
    m_orientation = orientation;
}

void Agent::computeGoalParameters()
{
    vec2 agentPosition = vec2(m_position.z    , m_position.x);
    vec2 agentVelocity = vec2(m_velocity.z    , m_velocity.x);
    vec2 goalPosition  = vec2(m_goalPosition.z, m_goalPosition.x);
    vec2 orientationv  = vec2(m_orientation.z , m_orientation.x);
    
    m_ttg = 1e30f;
    
    vec2 vectorFromAgentToGoal = goalPosition - agentPosition;
    
    m_distanceToGoal = glm::length(vectorFromAgentToGoal);
    vec2 goalDir     = glm::normalize(vectorFromAgentToGoal);

    // Projection of agentVelocity on goalDir.
    mFloat speedAlpha = glm::dot(agentVelocity, goalDir);
    
    // time = distance / speed.
    if (speedAlpha > 0.0)
        m_ttg = m_distanceToGoal / speedAlpha;
    
    // Vector from agent to goal after movement.
    vec2 goalDirN = glm::normalize(vectorFromAgentToGoal - (orientationv * m_comfortSpeed));
    
    // Bearing angle related to goal.
    mFloat alpha  = Utils::signedAngle(orientationv, goalDir);
    
    // Bearing angle related to goal after movement.
    mFloat alphaN = Utils::signedAngle(orientationv, goalDirN);
    
    // Time derivative of bearing goal
    mFloat alphaDotGoal = alphaN - alpha;
    
    // Ensures -PI < angle < PI
    if (alphaDotGoal > M_PI)
        alphaDotGoal -= 2 * M_PI;
    else if (alphaDotGoal < -M_PI)
        alphaDotGoal += 2 * M_PI;
    
    m_goalBearingAngleDerivative = alphaDotGoal;
    m_directionToGoal = vec3(goalDir.y, 0.0, goalDir.x);
}

vec3 Agent::getComfortVelocity()
{
    if (glm::length(m_goalPosition-m_position) < 0.2f)
       return m_directionToGoal * 0.0f;

    return m_directionToGoal * m_comfortSpeed;
}

const vec3& Agent::getGoalPosition() const
{
    return m_goalPosition;
}

void Agent::setGoalPosition(const vec3 &goalPosition)

{
    m_goalPosition = goalPosition;
}

mFloat Agent::getTimeToGoal() const
{
    return m_ttg;
}

mFloat Agent::getDistanceToGoal() const
{
    return m_distanceToGoal;
}

mFloat Agent::getGoalBearingAngleDerivative() const
{
    return m_goalBearingAngleDerivative;
}

mFloat Agent::getAngularVelocity() const
{
    return m_angularVelocity;
}

void Agent::setAngularVelocity(mFloat angularVelocity)
{
    m_angularVelocity = angularVelocity;
}

const vec3& Agent::getDirectionToGoal() const
{
    return m_directionToGoal;
}

void Agent::updateVelocityAndPosition(mFloat thetaMin, mFloat thetaMax, mFloat ttcMin, bool goFirstMin, bool goFirstMax, mFloat timeStep)
{
    mFloat speed = m_comfortSpeed;
    vec3 desiredVelocity;
    mFloat tti = ttcMin;
    bool goFirst = goFirstMin;
    
    computeGoalParameters();
    
    mFloat distToGoal = m_distanceToGoal;
    mFloat goalBearingAngleDerivative = m_goalBearingAngleDerivative;
    
    if (distToGoal < 0.2f * Utils::getMeter())
    {
        // at the goal, stop
        desiredVelocity = vec3(0.0f);
    }
    else if(tti >= 0.0f && tti < 10.0f)
    {
        mFloat currentSpeed = glm::length(getVelocity());
        mFloat ttcThreshold = 3.0f; // threshold for speed adaptation
        
        mFloat thetaDot = thetaMin;
        
        // Chose turning side according to the goal position
        // check if turning to the goal solves collisions
        if ( (thetaMin * goalBearingAngleDerivative > 0.0f && fabsf(thetaMin) < fabsf(goalBearingAngleDerivative)) ||
            (thetaMax * goalBearingAngleDerivative > 0.0f && fabsf(thetaMax) < fabsf(goalBearingAngleDerivative)))
        {
            thetaDot = goalBearingAngleDerivative;
        }
        else if (tti > 0.5f) // else avoid collisions
        {
            float t1 = fabsf(thetaMin - goalBearingAngleDerivative);
            float t2 = fabsf(thetaMax - goalBearingAngleDerivative);
            
            if ( (t1 < 0.08f && t2 < 0.08f) || t1 <= t2 )
            {
                thetaDot = thetaMin;
            } else
            {
                thetaDot = thetaMax;
                goFirst = goFirstMax;
            }
        }
        
        // giving way, slow down
        if (tti < ttcThreshold && !goFirst)
        {
            speed *= ((1 - pow(2.718f , -(tti*tti)*0.4f)));
            if (speed < 0.5f * Utils::getMeter()) speed = 0.0f;
        }
        
        // limit turning angle
        if (currentSpeed > 0.0f)
        {
            mFloat thetaDotOld = m_angularVelocity;
            mFloat tv = 0.5f;
            mFloat thetaDiff = Utils::clip(thetaDot - thetaDotOld, -tv, tv);
            thetaDot = thetaDotOld + thetaDiff;
        }
        
        mFloat angleInDegrees = glm::degrees(thetaDot);
        
        // prevents going from the goal
        vec3 newDir = glm::normalize(glm::rotateY(m_orientation, angleInDegrees));
        mFloat ldot = glm::dot(m_directionToGoal, newDir);
        
        if (currentSpeed != 0.0f && ldot < -0.8f && (tti != 0.0f))
        {
            thetaDot = 0.0f;
            angleInDegrees = 0.0f;
        }
        
        setAngularVelocity(thetaDot);
        
        mat4 rmat = glm::rotate(angleInDegrees, 0.0f, 1.0f, 0.0f);
        
        m_orientation = vec3(rmat * vec4(m_orientation, 1.0f));
        desiredVelocity = m_orientation * speed;
    }
    else
    {
        // No collision, use comfort velocity
        m_orientation = m_directionToGoal;
        desiredVelocity = getComfortVelocity();
        setAngularVelocity(0.0f);
    }
    
    if (distToGoal >= 0.2f)
    {
        vec3 oldVelocity = m_velocity;
        vec3 acceleration = desiredVelocity - oldVelocity;
        mFloat accLength = glm::length(acceleration);
        mFloat accLimit = timeStep * 1.0;
        if (accLength > accLimit)
        {
            vec3 accNorm = glm::normalize(acceleration);
            acceleration = accNorm * accLimit;
            desiredVelocity = oldVelocity + acceleration;
        }
    }
    
    if (desiredVelocity != vec3(0.0f))
    {
        m_orientation = glm::normalize(desiredVelocity);
    }
    
    setVelocity(desiredVelocity);
    
    vec3 pos = m_position + desiredVelocity * timeStep;
    setPosition(pos);
}
