/**
 * @file Agent.h
 * @author Teofilo Dutra
 */

#ifndef _AGENT_H_
#define _AGENT_H_

#include "definitions.h"
#include "Utils.h"

using namespace std;

class Agent
{
private:
    /** Center position. */
    vec3 m_position;
    
    /** Previous center position. */
    vec3 m_previousPosition;
    
    /** Velocity. */
    vec3 m_velocity;
    
    /** Previous velocity. */
    vec3 m_previousVelocity;
    
    /** Comfort speed. */
    mFloat m_comfortSpeed;
    
    /** Agent radius. */
    mFloat m_radius;
    
    /** Agent height. */
    mFloat m_height;
    
    /** Agent orientation. */
    vec3 m_orientation;
    
    /** Goal position. */
    vec3 m_goalPosition;
    
    /** Distance to goal. */
    mFloat m_distanceToGoal;
    
    /** Time to goal. */
    mFloat m_ttg;
    
    /** Time derivative of goal's bearing angle. */
    mFloat m_goalBearingAngleDerivative;
    
    /** Direction to goal. */
    vec3 m_directionToGoal;
    
    /** Agent's angular velocity. */
    mFloat m_angularVelocity;
    
public:
    
    /** Default constructor. */
    Agent();
    
    /** Copy constructor. */
    Agent(const Agent &agent);
    
    /** Destructor. */
    ~Agent() {}
    
    /**
     * Returns a clone of the agent.
     *
     * @returns Clone of the agent.
     */
    Agent *clone();
    
    /**
     * Returns the center position of the agent.
     *
     * @return Agent center position.
     */
    const vec3& getPosition() const;
    
    /**
     * Sets a new center position for the agent.
     *
     * @param position New position.
     */
    void setPosition(const vec3 &position);
    
    /**
     * Returns the previous center position of the agent.
     *
     * @return Agent previous center position.
     */
    const vec3& getPreviousPosition() const;
    
    /**
     * Sets a new previous center position for the agent.
     *
     * @param previousPosition New previous position.
     */
    void setPreviousPosition(const vec3 &previousPosition);
    
    /**
     * Returns the velocity of the agent.
     *
     * @return Agent velocity.
     */
    const vec3& getVelocity() const;
    
    /**
     * Sets a new velocity for the agent.
     *
     * @param velocity New velocity.
     */
    void setVelocity(const vec3 &velocity);
    
    /**
     * Returns the previous velocity of the agent.
     *
     * @return Agent previous velocity.
     */
    const vec3& getPreviousVelocity() const;
    
    /**
     * Sets a new previous velocity for the agent.
     *
     * @param previousVelocity New previous velocity.
     */
    void setPreviousVelocity(const vec3 &previousVelocity);
    
    /**
     * Returns the comfort speed of the agent.
     *
     * @return Agent comfort speed.
     */
    mFloat getComfortSpeed() const;
    
    /**
     * Sets a new comfort speed for the agent.
     *
     * @param comfortSpeed New comfort speed.
     */
    void setComfortSpeed(mFloat comfortSpeed);
    
    /**
     * Returns the agent radius.
     *
     * @return Agent radius.
     */
    mFloat getRadius() const;
    
    /**
     * Sets the new agent radius.
     *
     * @param radius New agent radius.
     */
    void setRadius(mFloat radius);
    
    /**
     * Returns the agent height.
     *
     * @return Agent height.
     */
    mFloat getHeight() const;
    
    /**
     * Sets the new agent height.
     *
     * @param radius New agent height.
     */
    void setHeight(mFloat height);
    
    /**
     * Returns the orientation of the agent.
     *
     * @return Agent orientation.
     */
    const vec3& getOrientation() const;
    
    /**
     * Sets a new orientation for the agent.
     *
     * @param orientation New orientation.
     */
    void setOrientation(const vec3 &orientation);
    
    /**
     * Computes goal parameters.
     */
    void computeGoalParameters();
    
    /**
     * Computes and returns comfort velocity.
     *
     * @return Agent's comfort velocity.
     */
    vec3 getComfortVelocity();
    
    /**
     * Returns the goal position.
     *
     * @return Goal position.
     */
    const vec3& getGoalPosition() const;
    
    /**
     * Sets a new goal position.
     *
     * @param goalPosition New goalPosition.
     */
    void setGoalPosition(const vec3 &goalPosition);
    
    /**
     * Returns the time to goal.
     *
     * @return Time to goal.
     */
    mFloat getTimeToGoal() const;
    
    /**
     * Returns ditance to goal.
     *
     * @return Distance to goal.
     */
    mFloat getDistanceToGoal() const;
    
    /**
     * Returns the time derivative of goal's bearing angle.
     *
     * @return Time derivative of goal's bearing angle.
     */
    mFloat getGoalBearingAngleDerivative() const;
    
    /**
     * Returns agent's angular velocity.
     *
     * @return Agent's angular velocity.
     */
    mFloat getAngularVelocity() const;
    
    /**
     * Sets a new angular velocity to the agent.
     *
     * @param angularVelocity New angular velocity.
     */
    void setAngularVelocity(mFloat angularVelocity);
    
    /**
     * Returns direction to goal.
     *
     * @return Direction to goal.
     */
    const vec3& getDirectionToGoal() const;
    
    /**
     * Updates velocity and position.
     */
    void updateVelocityAndPosition(mFloat thetaMin, mFloat thetaMax, mFloat ttc, bool goFirstMin, bool goFirstMax, mFloat timeStep);
};

#endif
