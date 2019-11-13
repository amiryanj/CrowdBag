/**
 *  @file CrowdModel.cpp
 *  @author Teofilo Dutra
 */

#include "CrowdModel.h"

CrowdModel::CrowdModel()
{
    m_textureWidth = 256;
    m_textureLength = 48;
    m_textureSize = m_textureWidth * m_textureLength;
    m_verticesPerCone = 20;
    m_totalVerticesPerCone = ((m_verticesPerCone - 2) * 3 * 2);
    m_verticesSize = m_totalVerticesPerCone * sizeof(vec4);
    
    m_texturePixels = new vec4[m_textureSize];
    m_coneVertices  = new vec4[m_verticesPerCone];
    m_vertices      = new vec4[m_totalVerticesPerCone];
    
    m_agentsData = NULL;
    m_agentsBuffer    = NULL;
    
    m_projectionMatrix = glm::frustum( -1.3f,
                                1.3f,
                                -0.60f,
                                0.01f,
                                0.2f,
                                100.0f);
    
    createCone();
    
    initializeOpenGL();
}

CrowdModel::~CrowdModel()
{
    m_agents.clear();
    
    SAFE_ARRAY_DELETE(m_vertices)
    SAFE_ARRAY_DELETE(m_texturePixels)
    SAFE_ARRAY_DELETE(m_coneVertices)
    SAFE_ARRAY_DELETE(m_agentsData)
    SAFE_ARRAY_DELETE(m_agentsBuffer)
}

void CrowdModel::doStep()
{
    if(!hasAgents())
    {
        printf("The simulator has no agents.");
        return;
    }
    
    computeAgentsPositions();
}

void CrowdModel::createTriangle(size_t &vertexIndex, size_t vertex1, size_t vertex2, size_t vertex3)
{
    m_vertices[vertexIndex] = m_coneVertices[vertex1];
    vertexIndex++;
    m_vertices[vertexIndex] = m_coneVertices[vertex2];
    vertexIndex++;
    m_vertices[vertexIndex] = m_coneVertices[vertex3];
    vertexIndex++;
}

void CrowdModel::createCone()
{
    m_coneVertices[0] = vec4(0.0f, 0.0f, 0.0f, 1.0f);
    m_coneVertices[1] = vec4(1.0f, 0.0f, 0.0f, 1.0f);
    
    mFloat angle = M_PI/(((m_verticesPerCone - 2)/2.0f));
    
    mat4 rotationMatrix = glm::rotate(glm::degrees(angle), 0.0f, 1.0f, 0.0f);
    
    for (int i = 2; i<m_verticesPerCone - 1; i++)
        m_coneVertices[i] = rotationMatrix * m_coneVertices[i-1];
    
    m_coneVertices[m_verticesPerCone-1] = vec4(0.0f, 1.0f, 0.0f, 1.0f);
    
    size_t vertexIndex = 0;
    
    // base
    for (int i=1; i < m_verticesPerCone - 1; i++)
    {
        if (i == m_verticesPerCone - 2)
            createTriangle(vertexIndex, 0, 1, i);
        else
            createTriangle(vertexIndex, 0, i+1, i);
    }
    
    // body
    for (int i=1; i < m_verticesPerCone - 1; i++)
    {
        if (i == m_verticesPerCone - 2)
            createTriangle(vertexIndex, i, 1, m_verticesPerCone - 1);
        else
            createTriangle(vertexIndex, i, i+1, m_verticesPerCone - 1);
    }
}

void CrowdModel::initializeTexture()
{

    glEnable(GL_TEXTURE_2D);

    // Create texture.
    glGenTextures(1, &m_texture);
    glBindTexture(GL_TEXTURE_2D, m_texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S    , GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T    , GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F_ARB, m_textureWidth, m_textureLength, 0, GL_RGBA, GL_FLOAT, NULL);
    
    // Create renderbuffer where texture will be rendered.
    glGenRenderbuffers(1, &m_rbo);
    glBindRenderbuffer(GL_RENDERBUFFER, m_rbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, m_textureWidth, m_textureLength);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, m_rbo);
    
    // Attach texture.
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_texture, 0);
    
    // Set the list of draw buffers.
    GLenum DrawBuffers[2] = {GL_COLOR_ATTACHMENT0};
    // "1" is the size of DrawBuffers.
    glDrawBuffers(1, DrawBuffers);
}

void CrowdModel::initializeShadersData()
{
    for (size_t programIndex = TEXTURE_PROGRAM; programIndex <= VISION_PROGRAM; programIndex++)
    {
        glUseProgram( m_program[programIndex] );
        
        // Create and bind the vertex array object.
#ifdef __APPLE__
        glGenVertexArraysAPPLE( 1, &m_vao[programIndex] );
        glBindVertexArrayAPPLE( m_vao[programIndex] );
#else
        glGenVertexArrays( 1, &m_vao[programIndex] );
        glBindVertexArray( m_vao[programIndex] );
#endif
        
        checkGLError();
        
        // Create and bind the buffer object.
        glGenBuffers( 1, &m_buffer[programIndex] );
        glBindBuffer( GL_ARRAY_BUFFER, m_buffer[programIndex] );
        
        checkGLError();
        
        // Initialize the vertex position attribute from the vertex shader.
        GLuint position_loc = glGetAttribLocation( m_program[programIndex], "vPosition" );
        
        glEnableVertexAttribArray( position_loc );
        glVertexAttribPointer( position_loc, 4, GL_FLOAT, GL_FALSE, 0, 0);
        glBufferData( GL_ARRAY_BUFFER, m_verticesSize, m_vertices, GL_STATIC_DRAW );

        glBindBuffer( GL_ARRAY_BUFFER, 0 );
#ifdef __APPLE__
        glBindVertexArrayAPPLE(0);
#else
        glBindVertexArray(0);
#endif
        
        checkGLError();
        
        // Get location of attributes.
        m_modelViewMatrixLocation[programIndex]  = glGetUniformLocation(m_program[programIndex], "model_view");
        m_projectionMatrixLocation[programIndex] = glGetUniformLocation(m_program[programIndex], "projection");
        m_agentVelocityLocation[programIndex]    = glGetUniformLocation(m_program[programIndex], "agentVelocity");
        m_agentPositionLocation[programIndex]    = glGetUniformLocation(m_program[programIndex], "agentPosition");
        m_agentRadiusLocation[programIndex]      = glGetUniformLocation(m_program[programIndex], "agentRadius");
        m_goalVelocityLocation[programIndex]     = glGetUniformLocation(m_program[programIndex], "goalVelocity");
        m_agentInstanceIndex[programIndex]       = glGetUniformLocation(m_program[programIndex], "agentInstanceIndex");
        m_agentsDataLocation[programIndex]       = glGetUniformLocation(m_program[programIndex], "pos_Vertex_Buff");
        m_ttgLocation[programIndex]              = glGetUniformLocation(m_program[programIndex], "ttg");
        
        checkGLError();

        // Send projection matrix.
        glUniformMatrix4fv(m_projectionMatrixLocation[programIndex], 1, GL_FALSE,  glm::value_ptr(m_projectionMatrix));
        
        checkGLError();
        
        glUseProgram(0);
    }
}

void CrowdModel::initializeOpenGL()
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    
    string path = "../SVModel/";
    
    m_program[TEXTURE_PROGRAM] = Utils::InitShader((path + "shaders/sv_vshader.vsh").c_str(), (path + "shaders/sv_fshader.fsh").c_str());
    m_program[VISION_PROGRAM]  = Utils::InitShader((path + "shaders/sv_vshader.vsh").c_str(), (path + "shaders/sv_visualization_fshader.fsh").c_str());
    
    glUseProgram(0);
    
    // Create and bind frame buffer object.
    glGenFramebuffers(1, &m_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
    
    initializeTexture();
    
    checkGLError();
    
    // Check if framebuffer is ok
    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    {
        printf("error.\n");
        return;
    }
    
    initializeShadersData();
    
    checkGLError();
    
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void CrowdModel::renderAgentVisionToTexture(size_t agentIndex, GLint programIndex)
{
    if ( programIndex < TEXTURE_PROGRAM ||
        programIndex > VISION_PROGRAM)
    {
        printf("Cannot render the vision for the program specified.");
        return;
    }
    
    if (agentIndex >= m_agents.size())
    {
        printf("Cannot render the vision because agent does not exist.");
        return;
    }
    
    Agent *agent = m_agents[agentIndex];
    
    glUseProgram(m_program[programIndex]);

#ifdef __APPLE__
    glBindVertexArrayAPPLE(m_vao[programIndex]);
#else
    glBindVertexArray(m_vao[programIndex]);
#endif
    
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
    
    glViewport(0, 0, m_textureWidth, m_textureLength);
    
    glClearColor(0.0f, 0.0f, -1.0f, -1.0f);
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    agent->computeGoalParameters();
    
    mFloat speed = glm::length(agent->getVelocity());
    
    // Prevents start walking if there is an obstacle in front of the agent.
    if (speed == 0.0f)
        speed = 0.1f * Utils::getMeter();
    
    vec3 agentOrientation = agent->getOrientation();
    vec3 agentPosition = agent->getPosition();
    vec3 agentVelocity = agentOrientation * speed;
    vec3 goalVelocity  = agent->getComfortVelocity();
    mFloat agentHeight = agent->getHeight();
    
    // Set camera to Agent's point of view
    vec3 eyePosition = agentPosition;
    vec3 at = eyePosition + agentOrientation;
    at.y = eyePosition.y = agentHeight;
    
    mat4 model_view = glm::lookAt(eyePosition, at, vec3(0,1,0));
    
    // Shader variables for viewer using uniforms.
    glUniformMatrix4fv(m_modelViewMatrixLocation[programIndex] , 1, GL_FALSE, glm::value_ptr(model_view));
    glUniform2fv( m_agentVelocityLocation[programIndex], 1, glm::value_ptr(vec2(agentVelocity.z, agentVelocity.x)));
    glUniform2fv( m_agentPositionLocation[programIndex], 1, glm::value_ptr(vec2(agentPosition.z, agentPosition.x)));
    glUniform2fv( m_goalVelocityLocation[programIndex],  1, glm::value_ptr(vec2(goalVelocity.z,  goalVelocity.x)));
    glUniform1f( m_agentRadiusLocation[programIndex], agent->getRadius());
    glUniform1f( m_ttgLocation[programIndex], agent->getTimeToGoal());
    glUniform1i( m_agentInstanceIndex[programIndex], (int)agentIndex);
    
    checkGLError();
    
    // Draw agents.
    glEnableClientState(GL_VERTEX_ARRAY);                    // Enable vertex arrays
    glBindBuffer(GL_ARRAY_BUFFER, m_buffer[programIndex]);   // Bind vertex buffer
    glVertexPointer(4, GL_FLOAT, 0, (char *) NULL);

    // Draw agents using hardware instancing.
    glDrawArraysInstancedARB(GL_TRIANGLES, 0, m_totalVerticesPerCone, (GLsizei) m_agents.size());
    glDisableClientState(GL_VERTEX_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    
    checkGLError();
    
    glutSwapBuffers();
    
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
#ifdef __APPLE__
    glBindVertexArrayAPPLE(0);
#else
    glBindVertexArray(0);
#endif
    glUseProgram(0);
}

void CrowdModel::updateAgentProperties(size_t agentIndex)
{
    size_t  startBufferIndex = agentIndex*5;
    bool    goFirstMin      = true;
    bool    goFirstMax      = true;
    mFloat  thetaMin        = 0.0f;
    mFloat  thetaMax        = 0.0f;
    mFloat  ttcMin          = -10.0f;
    mFloat  thetaDotPlus    = 0.0f;
    mFloat  thetaDotMinus   = 0.0f;
    bool    goFirstL        = true;
    bool    goFirstR        = true;
    
    mFloat thetaDotValue1   = m_agentsBuffer[startBufferIndex];                 // thetaDot
    mFloat thetaDotValue2   = m_agentsBuffer[startBufferIndex+1];               // thetaDot
    mFloat ttcValue         = m_agentsBuffer[startBufferIndex+2];               // ttc
    
    if(ttcValue >= 0.0f)
    {
        if(thetaDotValue1 < 10.0f)
        {
            thetaDotMinus = thetaDotValue1;
            thetaDotPlus  = thetaDotValue2;
        }
        ttcMin   = ttcValue;
        goFirstR = (m_agentsBuffer[startBufferIndex + 3] > 0.0f)? true: false;
        goFirstL = (m_agentsBuffer[startBufferIndex + 4] > 0.0f)? true: false;
    }
    
    if(fabsf(thetaDotMinus) > fabsf(thetaDotPlus))
    {
        thetaMin   = thetaDotPlus;
        thetaMax   = thetaDotMinus;
        goFirstMin = goFirstL;
        goFirstMax = goFirstR;
    }
    else
    {
        thetaMin   = thetaDotMinus;
        thetaMax   = thetaDotPlus;
        goFirstMin = goFirstR;
        goFirstMax = goFirstL;
    }
    
    m_agents[agentIndex]->updateVelocityAndPosition(thetaMin, thetaMax, ttcMin, goFirstMin, goFirstMax, m_timeStep);
}

void CrowdModel::computeAgentsPositions()
{
    for (size_t agentIndex = 0; agentIndex < m_agents.size(); agentIndex++)
    {
        Agent *agent = m_agents[agentIndex];
        
        vec3 agentPosition    = agent->getPosition();
        vec3 agentVelocity    = agent->getVelocity();
        vec3 agentOrientation = agent->getOrientation();
        
        // Storing agent data in a mat3x2 to pass it to the shaders.
        m_agentsData[agentIndex] = mat3x2(vec2(agentPosition.z, agentPosition.x), vec2(agentVelocity.z, agentVelocity.x), vec2(agentOrientation.z, agentOrientation.x));
    }
    
	// Values for static obstacles.
    m_agentsData[m_agents.size()] = mat3x2(0.0f);
    
    glUseProgram( m_program[TEXTURE_PROGRAM] );
    
    // Send data to the shader. Ondrej did this with UBO, but I changed because my graphics board does not support it.
    glUniformMatrix3x2fv(m_agentsDataLocation[TEXTURE_PROGRAM], (int) m_agents.size() + 1, false, glm::value_ptr(m_agentsData[0]) );
    
    for (size_t agentIndex = 0; agentIndex < m_agents.size(); agentIndex++)
    {
        renderAgentVisionToTexture(agentIndex, TEXTURE_PROGRAM);
        
        // ------------------- TODO: The code below must be implemented in cuda ---------------
        glBindTexture(GL_TEXTURE_2D, m_texture);
    	tic();
        glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_FLOAT, m_texturePixels);
        toc();
    
        mFloat phiR   = MFLOAT_MAX;
        mFloat phiL   = MFLOAT_MIN;
        mFloat tti    = MFLOAT_MAX;
        mFloat firstR = MFLOAT_MAX;
        mFloat firstL = MFLOAT_MAX;
        
        for (GLuint pixelIndex = 0; pixelIndex < m_textureSize; pixelIndex++)
        {
            vec4 pixel = m_texturePixels[pixelIndex];
            
            if(pixel.z >= 0.0f && pixel.x != 0.0f)
            {
                mFloat lPhiR   = pixel.x;     // thetaDot1
                mFloat lPhiL   = pixel.y;     // thetaDot2
                mFloat lTti    = pixel.z;     // ttc
                mFloat lFirstR = 1.0f;        // go first if taking thetaDot1
                mFloat lFirstL = 1.0f;        // go first if taking thetaDot2
                
                if (pixel.z < 3.0f && pixel.w <= 0.0f) // ttc < 3s and giving a way
                {
                    if (fabsf(pixel.x) < fabsf(pixel.y))
                    {
                        lFirstR = -1.0f;
                    } else
                    {
                        lFirstL = -1.0f;
                    }
                }
                
                phiR   =    Utils::min(phiR,    lPhiR);
                phiL   =    Utils::max(phiL,    lPhiL);
                tti    =    Utils::min(tti,     lTti);
                firstR =    Utils::min(firstR,  lFirstR);
                firstL =    Utils::min(firstL,  lFirstL);
            }
        }
        
        if (tti == MFLOAT_MAX)
            tti = -1.0f;
        
        size_t startBufferIndex = agentIndex * 5;
        m_agentsBuffer[startBufferIndex]   = phiR;
        m_agentsBuffer[startBufferIndex+1] = phiL;
        m_agentsBuffer[startBufferIndex+2] = tti;
        m_agentsBuffer[startBufferIndex+3] = firstR;
        m_agentsBuffer[startBufferIndex+4] = firstL;
        // ------------------- TODO: The code above must be implemented in cuda ---------------
    }
    
    for (size_t agentIndex = 0; agentIndex < m_agents.size(); agentIndex++)
    {
        updateAgentProperties(agentIndex);
    }
}

mFloat CrowdModel::getTimeStep() const
{
    return m_timeStep;
}

void CrowdModel::setTimeStep(mFloat timeStep)
{
    m_timeStep = timeStep;
}

bool CrowdModel::hasAgents() const
{
    return (m_agents.size() > 0);
}

const vector<Agent*>& CrowdModel::getAgents() const
{
    return m_agents;
}

void CrowdModel::setAgents(const vector<Agent*>& agents)
{
    m_agents.clear();
    m_agents = agents;
    
    SAFE_ARRAY_DELETE(m_agentsData)
    
    m_agentsData = new mat3x2[agents.size() + 1];
    
    SAFE_ARRAY_DELETE(m_agentsBuffer)
    
    m_agentsBuffer = new mFloat[5 * agents.size()];
}

void CrowdModel::renderVisionForVisualization(size_t agentIndex)
{
    if (agentIndex >= m_agents.size())
        return;
    
    glUseProgram( m_program[VISION_PROGRAM] );
    
    glUniformMatrix3x2fv(m_agentsDataLocation[VISION_PROGRAM], (int) m_agents.size()+1, false, glm::value_ptr(m_agentsData[0]) );
    
    //glUniform4fv( m_agentsDataLocation[VISION_PROGRAM], (int) m_agents.size() + 1, value_ptr(m_agentsPositions[0]));
    
    renderAgentVisionToTexture(agentIndex, VISION_PROGRAM);
    
    // glBindTexture(GL_TEXTURE_2D, m_texture);
    // glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_FLOAT, m_visionPixels);
}

void CrowdModel::checkGLError()
{
    GLint error = glGetError();
    
    switch (error) {
        case GL_INVALID_ENUM:
            printf("OpenGL Error: Invalid enum.");
            break;
        case GL_INVALID_VALUE:
            printf("OpenGL Error: Invalid value.");
            break;
        case GL_INVALID_OPERATION:
            printf("OpenGL Error: Invalid operation.");
            break;
        case GL_INVALID_FRAMEBUFFER_OPERATION:
            printf("OpenGL Error: Invalid frame buffer operation.");
            break;
        case GL_OUT_OF_MEMORY:
            printf("OpenGL Error: Out of memory.");
            break;
        case GL_STACK_OVERFLOW:
            printf("OpenGL Error: Stack overflow.");
            break;
        case GL_STACK_UNDERFLOW:
            printf("OpenGL Error: Stack underflow.");
            break;
        default:
            break;
    }
    
    if  (error != GL_NO_ERROR)
        getchar();
}
