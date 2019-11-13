/**
 *  @file CrowdModel.h
 *  @author Teofilo Dutra
 */

#ifndef _CROWDMODEL_H_
#define _CROWDMODEL_H_

#include "definitions.h"
#include "Agent.h"
#include "Utils.h"
#include <vector>

using namespace std;

class CrowdModel
{
private:
    /** Indexes of programs used. */
    enum
    {
        TEXTURE_PROGRAM = 0, /** Relates to the program used to get the data to compute the agents' velocities. */
        VISION_PROGRAM /** Relates to the program used to render the vision for visualization purporses. */
    };
    
    /** Time step of the simulation. */
    mFloat m_timeStep;
    
    /** Array of agents. */
    vector<Agent*> m_agents;
    
    /** Projection matrix for rendering vision. */
    mat4 m_projectionMatrix;
    
    GLuint m_textureWidth,         /** Vision texture width. */
           m_textureLength,        /** Vision texture length. */
           m_textureSize,          /** Vision texture size. */
           m_verticesPerCone,      /** Number of vertices in the cone. */
           m_totalVerticesPerCone; /** Number of vertices used to render the cone. */

    vec4 *m_vertices,       /** Array of vertices used to render the cone. */
         *m_texturePixels,  /** Array of texture pixels. */
         *m_coneVertices;   /** Array of the cone vertices. */
    
    /** Memory size (bytes) of the array of vertices of the cone. */
    size_t m_verticesSize;
    
    /** OpenGL stuff. */
    GLuint m_program[2],
           m_vao[2],
           m_buffer[2],
           m_fbo,
           m_rbo,
           m_texture;
    
    /** OpenGL stuff. */
    GLint  m_modelViewMatrixLocation[2],
           m_projectionMatrixLocation[2],
           m_agentVelocityLocation[2],
           m_agentPositionLocation[2],
           m_agentRadiusLocation[2],
           m_goalVelocityLocation[2],
           m_agentInstanceIndex[2],
           m_agentsDataLocation[2],
           m_ttgLocation[2];
    
    /** Used to store the agents' data to be sent to shaders. */
    mat3x2 *m_agentsData;
    
    /** Temporary. */
    mFloat *m_agentsBuffer;
    
    /**
     * Creates a triangle in m_vertices with 3 vertices of m_coneVertices.
     *
     * @param vertexIndex Index in m_vertices where the first vertex of the triangle will be placed.
     * @param vertex1 First vertex.
     * @param vertex2 Second vertex.
     * @param vertex3 Third vertex.
     */
    void createTriangle(size_t &vertexIndex, size_t vertex1, size_t vertex2, size_t vertex3);
    
    /**
     * Creates the cone that is used to represent the agents.
     */
    void createCone();
    
    /**
     * Initializes the texture used for vision.
     */
    void initializeTexture();
    
    /**
     * Initializes the shaders.
     */
    void initializeShadersData();
    
    /**
     * Initializes OpenGL stuff.
     */
    void initializeOpenGL();
    
    /**
     * Displays OpenGL errors, if any.
     */
    void checkGLError();
    
    /**
     * Renders agent vision to a texture.
     *
     * @param agentIndex Index of the agent whose vision will be rendered.
     * @param programIndex Program index.
     */
    void renderAgentVisionToTexture(size_t agentIndex, GLint programIndex);
    
    /**
     * Computes the agents' positions.
     */
    void computeAgentsPositions();
    
    /**
     * Updates agent properties (orientation, position, velocity, ...).
     */
    void updateAgentProperties(size_t agentIndex);
    
public:
    /** Default constructor. */
    CrowdModel();
    
    /** Destructor. */
    ~CrowdModel();
    
    /**
     * Performs a step, updating the agents, dynamic obstacles and etc.
     */
    void doStep();
    
    /**
     * Returns the time step.
     *
     * @return Time step.
     */
    mFloat getTimeStep() const;
    
    /**
     * Sets a new time step.
     *
     * @param timeStep New time step.
     */
    void setTimeStep(mFloat timeStep);
    
    /**
     * Returns if the instance has agents.
     *
     * @return True, if it has agents; otherwise, false.
     */
    bool hasAgents() const;
    
    /**
     * Returns the array of agents.
     *
     * @return Array of agents.
     */
    const vector<Agent*>& getAgents() const;
    
    /**
     * Sets a new array of agents.
     *
     * @param agents New array of agents.
     */
    void setAgents(const vector<Agent*>& agents);
    
    /**
     * Renders the vision of the specified agent for visualization purporses.
     *
     * @param agentIndex Agent index.
     */
    void renderVisionForVisualization(size_t agentIndex);
};


#endif
