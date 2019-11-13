
#include "../simulations/mymath/include/mat2.h"
#include "../include/pedestrianVision.h"

namespace craal {

#define Pi 3.141592653589793
#define WAIT_TIME 10

using namespace std;

std::vector<Seg2> PedestrianV::_sobstacles;

PedestrianV::PedestrianV()
{
    restart();
    name = "Vision";
}

PedestrianV::PedestrianV(Vec2 position, Vec2 goal, float speedComfort,
                         float speedMax, float personalArea)
{
    init();
    init(position, goal, speedComfort, speedMax, personalArea);
}

PedestrianV::~PedestrianV()
{
}

void PedestrianV::restart()
{
    init();
    init(Vec2(0.0,0.0), Vec2(0.0,0.0), 1.5, 2.0, 0.5);
}

void
PedestrianV::addObstacle(Seg2 o)
{
    _sobstacles.push_back(o);
}

void
PedestrianV::addPedestrianInteraction(Pedestrian *pedPt)
{
    PedestrianVInteraction newInteraction;
    newInteraction._pedPt = (PedestrianV*)pedPt;
    newInteraction._personalAreaRadius = ((PedestrianV*)pedPt)->_personalAreaRadius + _personalAreaRadius;
    newInteraction._visible = true;
    newInteraction._timeObservation = 0;
    newInteraction._id = _intPedCount++;


    _pedestrianInteractions.push_back(newInteraction);
}

bool
PedestrianV::checkVisibility(const PedestrianV &ped)
{
    bool ret = true;
    Seg2 pos(_position, ped._position);

    for (unsigned int i = 0; i < _sobstacles.size(); i++)
    {
        if ( pos.doIntersect(_sobstacles[i]) )
        {
            return false;
        }
    }

    return ret;
}


bool
PedestrianV::computeSolutionToInteraction(PedestrianVInteraction &pedInt)
{

    return true;
}

void
PedestrianV::computeRelativeRepresentation(PedestrianVInteraction &pedInt)
{
    Mat2 matRot;
    PedestrianV *ped = (pedInt)._pedPt;

    pedInt._matRot.setRot(-atan2(_orientation.y, _orientation.x));

    Vec2 relativeVelocity   =  pedInt._matRot * (Vec2(0.0, 0.0) - _velocity);
    pedInt._relativePosition = pedInt._matRot * (ped->_position - _position);
    pedInt._relativeVelocity = pedInt._matRot * (ped->_velocity);
    pedInt._relativeComposedVelocity = pedInt._relativeVelocity + relativeVelocity;

    //    if (_velocity.longueur() == 0)
    //    {
    //        Vec2 relativeVelocity1   = pedInt._matRot * (Vec2(0.0, 0.0) - _orientation*0.3);
    //        pedInt._relativeComposedVelocity = pedInt._relativeVelocity + relativeVelocity1;
    //    }

    //    /// orientation change
    //    float o1 = atan2(ped->_velocity.y, ped->_velocity.x);       // current orientation
    //    float o2 = atan2(pedInt._velocity.y, pedInt._velocity.x);   // old orientation
    //    if(o1 < -2.0 && o2 > 2.0)  o1 += 6.28;
    //    if(o2 < -2.0 && o1 > 2.0)  o2 += 6.28;
    //
    //    float oc = (o1 - o2)/_dt;
    /// for oscilation or big change set to 0
    //    if( (oc < 0 && pedInt._oriChange > 0) || (oc > 0 && pedInt._oriChange < 0) )
    //        pedInt._oriChange = 0;
    //    else if(fabs(oc) > 0.30)
    //        pedInt._oriChange = 0;
    //    else
    //        pedInt._oriChange = oc;

    pedInt._velocity = ped->_velocity;
    pedInt._orientation = ped->_orientation;

    //    if (relativeVelocity.longueur() == 0 || pedInt._relativeVelocity.longueur() == 0)
    //    {
    //        pedInt._relativeDirectionAngle = 0;
    //    }
    //    else
    //    {
    //        pedInt._relativeDirectionAngle = acos( (float) ((relativeVelocity
    //                                               * pedInt._relativeVelocity)/(relativeVelocity.longueur()
    //                                                                            * pedInt._relativeVelocity.longueur()))
    //                                             );
    //    }

}

//void tangentsToSphere(Vec2 p, float r)
//{
//    float m = p.x;
//    float n = p.y;
//    float C = m*m +  n*n - r*r;
//
//    float a1 =  ( n * m + sqrt(n * n * C + C * m * m - C * C)) / (-n * n + C);
//    float a2 = -(-n * m + sqrt(n * n * C + C * m * m - C * C)) / (-n * n + C);
//
//    float x1 =  (  m + (n * a1) ) / (1 + a1 * a1);
//    float x2 =  (  m + (n * a2) ) / (1 + a2 * a2);
//
//    float y1 = a1 * x1;
//    float y2 = a2 * x2;
//}

void
PedestrianV::computeTimeToContact(PedestrianVInteraction &pedInt)
{
    Mat2 matRot;

    pedInt._ttc = PED_INF;
    pedInt._timeToCollision = PED_INF;
    pedInt._distanceAtCollison = PED_INF;
    float distance = PED_INF;
    float dist = PED_INF;

    //    Line relativeTraj (pedInt._relativeComposedVelocity, pedInt._relativePosition);
    //    float mpd = relativeTraj.distanceToPoint (Vec2 (0.,0.));
    float personalAreaRadius = _personalAreaRadius + pedInt._pedPt->_personalAreaRadius;
    distance = max(0.01f, pedInt._relativePosition.longueur() - personalAreaRadius);
    dist = pedInt._relativePosition.longueur();
    pedInt._distance = distance;

    float alpha = atan2(pedInt._relativePosition.y, pedInt._relativePosition.x);
    pedInt._alpha = alpha;

    if (pedInt._visible && pedInt._relativePosition.x >= -0.2 )
    {
        Vec2 relPosDir = Vec2() - pedInt._relativePosition;
        relPosDir.normalize();

        float st = relPosDir * pedInt._relativeComposedVelocity;
        if( distance == 0)
            st = 1e30;

        if (st > 0)
        {
            float ttc = max(0.000001f, distance / st);

            Vec2 newpos = pedInt._relativePosition + pedInt._relativeComposedVelocity;
            float alphaNext = atan2(newpos.y, newpos.x);
            float alphaDot = alphaNext - alpha;
            if(fabs(alphaDot) > 3.14)
            {
                if(alphaDot > 3.14)
                    alphaDot -= 6.28;
                else
                    alphaDot += 6.28;
            }
            if(alphaDot > 1.57)
                alphaDot = 3.14 - alphaDot;
            else if(alphaDot < -1.57)
                alphaDot = -3.14 - alphaDot;

            pedInt._alpha = alpha;
            pedInt._alphaDot = alphaDot;
            //            pedInt._ttc = (dist / st);
            pedInt._ttc = (distance / st);
            pedInt._timeToCollision = ttc;
        }
    }
}

/// compute comfort velocity to the goal or to the next waypoint
void
PedestrianV::computeGoalPoint()
{
    if(!_pathGateway[_nextWaypoint])
    {
        _goal = _path[0][_nextWaypoint];
    }
    else
    {
        Line l1, l2;
        l1.defineBy2Points(_path[0][_nextWaypoint], _path[1][_nextWaypoint]);
        l2.defineByPerpLinePoint(l1, _position);

        Vec2 p = l1.intersection(l2);
        Vec2 dir1 = _path[1][_nextWaypoint] - _path[0][_nextWaypoint];
        Vec2 dir2 = p - _path[0][_nextWaypoint];

        float dot = (float) (dir1 * dir2);
        if(dot <= 0)
        {
            p = _path[0][_nextWaypoint];
        }
        else if(dir2.longueur() > dir1.longueur())
        {
            p = _path[1][_nextWaypoint];
        }
        _goal = p;
    }
}

void
PedestrianV::computeComfortVelocity()
{
    if (_direction)
    {
        _velocityComfort = _goal * g_params[3];
        g_distToGoal = 1e29;
        _dirToGoal = _goal;
        _dirToGoal.normalize();
    }
    else if(_directionCircular)
    {
        Vec2 dir = _position - _goal;
        float distToCenter = dir.normalize();

        if(_directionCircularCCW)
            _dirToGoal = Vec2(-dir.y, dir.x);
        else
            _dirToGoal = Vec2(dir.y, -dir.x);

        // comfort velocity
        _velocityComfort = _dirToGoal * g_params[3];

        // corection
        Vec2 newPos = _position + _velocityComfort * _dt;
        dir = newPos - _goal;
        float d = distToCenter - dir.normalize();

        _dirToGoal = (newPos + dir*d - _position);
        _dirToGoal.normalize();

        _velocityComfort = _dirToGoal * g_params[3];
        g_distToGoal = 1e29;
    }
    else
    {
        computeGoalPoint();
        _dirToGoal = _goal - _position;
        g_distToGoal = _dirToGoal.normalize();
        _velocityComfort = _dirToGoal * g_params[3];
    }
}

float PedestrianV::getComfortSpeed()
{
    return g_params[3];
}
bool PedestrianV::getCircleCCW()
{
    return _directionCircularCCW;
}
bool PedestrianV::getCirclePath()
{
    return _directionCircular;
}
bool PedestrianV::getInfPath()
{
    return _infPath;
}
float PedestrianV::getPersonalArea()
{
    return _personalAreaRadius;
}
Vec2 PedestrianV::getGoal()
{
    return _goal;
}
Vec2 PedestrianV::getOrientation()
{
    return _orientation;
}
Vec2 PedestrianV::getOrientationNew()
{
    return _orientationNew;
}
vector<Vec2> PedestrianV::getPath()
{
    return _path[0];
}
Vec2 PedestrianV::getPosition()
{
    return _position;
}
Vec2 PedestrianV::getPositionNew()
{
    return _positionNew;
}
bool PedestrianV::getVisible()
{
    bool ret = false;
    list<PedestrianVInteraction>::iterator pedInterIt = _pedestrianInteractions.begin();
    list<PedestrianVInteraction>::iterator pedInterItEnd = _pedestrianInteractions.end();

    if (pedInterIt != pedInterItEnd)
    {
        ret = (*pedInterIt)._visible;
    }

    return ret;
}

Vec2 PedestrianV::getVelocity()
{
    return _velocity;
}

string
PedestrianV::info()
{
    cout << "Position: " << _position << endl
         << "Target: " << _goal << endl
         << "Max Speed: " << _speedMax << endl
         << "Comfort Speed: " << g_params[3] << endl
         << "Personal Radius: " << _personalAreaRadius << endl
         << endl;

    return " ";
}

void
PedestrianV::init(Vec2 position, Vec2 goal, float speedComfort, float speedMax,
                  float personalArea)
{
    _position = position;
    _positionNew = position;
    _goal = goal;
    _direction = false;
    _directionCircular = false;
    _directionCircularCCW = false;
    _speedMax = speedMax;
    g_params[3] = speedComfort;
    _personalAreaRadius = personalArea;
    _velocity = Vec2(0.0,0.0);
    _orientation = _goal - _position;
    _orientation.normalize();


    setDefaultPath();
    _infPath = false;

    computeComfortVelocity();
    _velocityDesired = _velocityComfort;
}

void
PedestrianV::init()
{
    _accelerationMax = 0.90;
    //_acceleration = 0.0;
    _direction = false;
    _directionCircular = false;
    _directionCircularCCW = false;
    pedInterToSolve = 0;
    _intPedCount = 0;
    _start = 0;
    _thetaDotOld = 0;
    _waitTime = 0.0;
    _waiting = false;
    _print = false;
    _p2 = 0.8;
    _q2 = 2.0;
    g_params[3] = 1.5;
    _personalAreaRadius = 0.2;

    // Obstacles for experiments
    // TODO: replace with addObstacle function
}

void
PedestrianV::reset()
{
    _accelerationMax = 0.90;
    //_acceleration = 0.0;
    _direction = false;
    _directionCircular = false;
    _directionCircularCCW = false;
    _start = 0;
    _thetaDotOld = 0;
    _waitTime = 0.0;
    _waiting = false;
    _print = false;
    _p2 = 0.8;
    _q2 = 2.0;

    _position = Vec2(0.0,0.0);
    _positionNew = Vec2(0.0,0.0);
    _goal = Vec2(0.0,0.0);
    _direction = false;
    _directionCircular = false;
    _directionCircularCCW = false;
    _speedMax = 2.0;
    g_params[3] = 1.5;
    _personalAreaRadius = 0.2;
    _velocity = Vec2(0.0,0.0);
    _orientation = _goal - _position;
    _orientation.normalize();


    setDefaultPath();
    _infPath = false;

    computeComfortVelocity();
    _velocityDesired = _velocityComfort;

    //     _position = _path[0][0];
    //     _positionNew = _path[0][0];
    //     _velocity = Vec2();
    //     _nextWaypoint = 1;
    //
    //     computeComfortVelocity();
    //     _velocityDesired = _velocityComfort;
    //
    //     _start = 0;
}

void PedestrianV::addObstacle(float s_startx, float s_starty, float s_endx, float s_endy)
{
    Vec2 v1(s_startx, s_starty);
    Vec2 v2(s_endx, s_endy);
    Seg2 s(v1, v2);
    addObstacle(s);
}

void PedestrianV::setComfortSpeed(float cs)
{
    g_params[3] = cs;
}
void PedestrianV::setGoal(Vec2 g, bool dir)
{
    _goal = g;
    _direction = dir;
    if (_direction) _goal.normalize();
    setDefaultPath();
}

void PedestrianV::setDefaultPath()
{
    _path[0].resize(2);
    _path[1].resize(2);
    _path[0][0] = _position;
    _path[1][0] = _position;
    _path[0][1] = _goal;
    _path[1][1] = _goal;
    _pathGateway.push_back(false);
    _pathGateway.push_back(false);
    _nwaypoints = 2;
    _nextWaypoint = 1;

    computeComfortVelocity();
    _velocityDesired = _velocityComfort;
}

void PedestrianV::setPrint(bool b)
{
    _print = b;
}

void
PedestrianV::addPath(Vec2 path1, bool inf)
{
    addPath(path1, path1, false, inf);
}

void
PedestrianV::addPath(Vec2 path1, Vec2 path2, bool gateway, bool inf)
{
    int size = (int) _path[0].size();

    _path[0].push_back(path1);
    _path[1].push_back(path2);
    _pathGateway.push_back(gateway);
    _nwaypoints = _path[0].size();
    _infPath = inf;

    if(size == 0)
    {
        // first point of the path is starting position
        if(gateway)
        {
            _position =    (_path[0][0] + _path[1][0]) * 0.5;
            _positionNew = (_path[0][0] + _path[1][0]) * 0.5;
        }
        else
        {
            _position =    _path[0][0];
            _positionNew = _path[0][0];
        }
        _direction = false;
        _directionCircular = false;
        _directionCircularCCW = false;
        _nextWaypoint = 0;
        _orientation = Vec2(1.0,0.0);
    }
    else
    {
        _orientation =  _path[0][1] - _position;
        _orientation.normalize();
        _nextWaypoint = 1;
    }
    _goal =  (_path[0][_nextWaypoint] + _path[1][_nextWaypoint]) * 0.5;
}

void
PedestrianV::setPath(const vector<Vec2>& path, bool inf)
{
    int size = (int) path.size();
    if(size == 0)
    {
        // reset
        _path[0].clear();
        _path[1].clear();
        _pathGateway.clear();
    }
    if ( size >= 2 )
    {
        _path[0].clear();
        _path[1].clear();
        _path[0].resize(size);
        _path[1].resize(size);
        _pathGateway.resize(size);
        for(int i = 0; i < size; i++)
        {
            _path[0][i] = path[i];
            _path[1][i] = path[i];
            _pathGateway[i] = false;
        }
        _goal = _path[0][size-1];

        _direction = false;
        _directionCircular = false;
        _directionCircularCCW = false;
        _position = _path[0][0];
        _positionNew = _path[0][0];
        _nwaypoints = size;
        _nextWaypoint = 1;
        _infPath = inf;
        _orientation =  _path[0][1] - _position;
        _orientation.normalize();
    }
}

void PedestrianV::setCircleCCW(bool b)
{
    _directionCircularCCW = b;
}

void PedestrianV::setCirclePath(bool b)
{
    _directionCircular = b;
}

void PedestrianV::setInfPath(bool b)
{
    _infPath = b;
}

void
PedestrianV::setPersonalArea(float r)
{
    _personalAreaRadius = r;
}

void PedestrianV::setStartPosition(Vec2 p)
{
    _position = p;
    _positionNew = p;
    setDefaultPath();
}

void PedestrianV::setPosition(Vec2 p)
{
    _position = p;
}

void PedestrianV::setVelocity(Vec2 v)
{
    _velocity = v;
}
void PedestrianV::setVisible(PedestrianV *pedPt, bool v)
{
}

/// Soleve interactions, compute new velocity and choose new goal
void
PedestrianV::solveInteractions()
{
    float clippedSpeed;
    solvePedestrianVInteractions();

    if ( !_direction && !_directionCircular && ( _nextWaypoint == _nwaypoints-1 || _nextWaypoint == 0) )
    {
        // going to goal
        float slowingDistance = max(1.0f, 1.3f * _velocity.longueur());
        float speed = _velocityDesired.longueur();
        clippedSpeed = pow(g_distToGoal / slowingDistance,2);
        clippedSpeed = speed*clippedSpeed;
        clippedSpeed = std::min(speed, clippedSpeed);
        clippedSpeed = min(2.0f, clippedSpeed);

        if (_infPath && g_distToGoal < 1.0) _nextWaypoint = (_nextWaypoint+1)%_nwaypoints;
    }
    else
    {
        // going to waypoint
        clippedSpeed = _velocityDesired.longueur();
        if (g_distToGoal < 1.0) _nextWaypoint = (_nextWaypoint+1)%_nwaypoints;
    }

    _velocityDesired.normalize();
    _velocityDesired = _velocityDesired * clippedSpeed;
}



bool compareTTC(PedestrianVInteraction *first, PedestrianVInteraction *second)
{
    return (first->_timeToCollision < second->_timeToCollision);
}

bool compareTTC1(PedestrianVInteraction &first, PedestrianVInteraction &second)
{
    return (first._timeToCollision < second._timeToCollision);
}

bool compareDist(PedestrianVInteraction *first, PedestrianVInteraction *second)
{
    return (first->_distance < second->_distance);
}

bool compareDist1(PedestrianVInteraction &first, PedestrianVInteraction &second)
{
    return (first._distance < second._distance);
}

bool compareAlpha(PedestrianVInteraction *first, PedestrianVInteraction *second)
{
    return (fabs(first->_alpha) < fabs(second->_alpha));
}


/// Solve interaction with pedestrians
void
PedestrianV::solvePedestrianVInteractions()
{
    list<PedestrianVInteraction>::iterator pedInterIt = _pedestrianInteractions.begin();
    list<PedestrianVInteraction>::iterator pedInterItEnd = _pedestrianInteractions.end();
    list<PedestrianVInteraction*> lPedInterToSolve;
    PedestrianVInteraction* pedMinTTC = 0;

    // Find all interacting pedestrians
    float minttc = 1e30;
    float ttc = 10;
    float ttg = 1e29;
    float mindtc = 1e29;
    Vec2 zero(0.0,0.0);

    //     float p1 = 0.65;
    //     float q1 = 1.5;
    //     float c1 = -0.01;
    //
    //     float p2 = 0.75;
    //     float q2 = 1.5;
    //     float c2 = -0.01;

    // 	std::cout<< *a<< " "<< *b<< " "<< *c<<  std::endl;

    float p1 = g_params[1];
    float q1 = g_params[2];
    float c1 = g_params[0];

    float p2 = g_params[1];
    float q2 = g_params[2];
    float c2 = g_params[0];

    // 	std::cout<< c2<< " "<< p2<< " "<< q2<< std::endl;

    //    int i = 0;
    /// find colliding pedestrians and chose the one with smallest ttc
    while (pedInterIt != pedInterItEnd)
    {
        computeRelativeRepresentation((*pedInterIt));
        pedInterIt->_collision = 0;
        //        pedInterIt->_distance = (_position - pedInterIt->_pedPt->_position).longueur();
        computeTimeToContact((*pedInterIt));

        /// check visibility
        //        pedInterIt->_distance = pedInterIt->_distance - pedInterIt->_pedPt->_personalAreaRadius - _personalAreaRadius;

        if (pedInterIt->_distance < mindtc)
            mindtc = pedInterIt->_distance;

        float v = c2 + p2/(float) pow((float)pedInterIt->_timeToCollision, (float)q2);
        float Td = min(max(0.0f, v), 1.57f);

        // 		if (v > 100 || v < -100) {std::cout<< "v"<< (float)pedInterIt->_timeToCollision<< "\n";}

        if (pedInterIt->_visible && fabs(pedInterIt->_alphaDot) < Td)
        {
            pedInterToSolve = &(*pedInterIt);
            lPedInterToSolve.push_back(&(*pedInterIt));

            if (pedInterIt->_timeToCollision < minttc)
            {
                minttc = pedInterIt->_timeToCollision;
                pedMinTTC = &(*pedInterIt);
            }
        }
        ++pedInterIt;
        //        ++i;
    }

    /// Goal parameters
    _alphaDotGoal = 0;
    float alphaGoal;
    if( !(_direction || _directionCircular) )
    {
        /// Time to goal
        if (_velocity.longueur() > 0 )
        {
            float st2g = _velocity * _dirToGoal;  // tangent speed to goal
            ttg = g_distToGoal/st2g;
        }

        /// AlphaDot for goal
        Vec2 relativeGoal = _goal - _position;
        Vec2 relVel = (Vec2(0.0, 0.0) - (_orientation * g_params[3]));
        Vec2 relativeGoalNext = relativeGoal + relVel;

        alphaGoal =     atan2(relativeGoal.y, relativeGoal.x);
        float alphaGoalNext = atan2(relativeGoalNext.y, relativeGoalNext.x);

        _alphaDotGoal = alphaGoalNext - alphaGoal;
        if(_alphaDotGoal < -3.14)
            _alphaDotGoal += 6.28;
        if(_alphaDotGoal > 3.14)
            _alphaDotGoal -= 6.28;

        float dot = relativeGoal * _orientation;
        if(dot < 0.0)
            _alphaDotGoal += (_alphaDotGoal > 0.0) ? 1.57 : -1.57;
    }
    else
    {
        Mat2 mrot;
        mrot.setRot(-atan2(_orientation.y, _orientation.x));
        Vec2 relativeGoal = mrot * _velocityComfort;
        alphaGoal = atan2(relativeGoal.y, relativeGoal.x);

        _alphaDotGoal = alphaGoal*0.5;
    }

    if (_nextWaypoint < _nwaypoints-1) ttg = 1e29;
    if (_velocity.longueur() == 0 ) ttg = 1e30;

    if (g_distToGoal < 1.0 && mindtc < 0.10) // Someone is standing at the goal. Stop!
    {
        _velocityDesired = Vec2();
    }
    else if(_waitTime > 0.0)
    {
        _waitTime -= _dt;
    }
    else
    {
        /// Solve collisions
        if ( (ttc >= 0) && (minttc < ttc) && (minttc < ttg) && (pedMinTTC != 0) ) // Solve collision
        {
            float thetaDot = 0;
            float speed = g_params[3];

            lPedInterToSolve.sort(compareDist);
            list<PedestrianVInteraction*>::iterator pedInterIt = lPedInterToSolve.begin();
            list<PedestrianVInteraction*>::iterator pedInterItEnd = lPedInterToSolve.end();

            bool  first = true;
            bool  firstR = true;
            bool  firstL = true;
            float ttcTreshlod = 3.0;
            float thetaDotPlus = 0.0;
            float thetaDotMinus = 0.0;

            while (pedInterIt != pedInterItEnd)
            {
                PedestrianVInteraction *pedInterToSolve = (*pedInterIt);

                float ttc = pedInterToSolve->_timeToCollision;
                float alphaDot = pedInterToSolve->_alphaDot;
                float alpha = pedInterToSolve->_alpha;
                float v;

                bool second = false;
                if( ttc <= 3 && alpha * alphaDot <= 0.0)  second = true;

                if( pedInterToSolve->_pedPt->_velocity.longueur() == 0 || second )
                    v = c2 + p2/pow(ttc, q2);
                else
                    v = c1 + p1/pow(ttc, q1);

                float Td = min(max(0.0f, v), 1.57f);
                if(ttc < ttg  && fabs(alphaDot) < Td )
                {
                    float thetaDot1 = -Td + alphaDot; // -
                    float thetaDot2 =  Td + alphaDot; // +

                    if(fabs(thetaDot1) < fabs(thetaDot2))
                    {
                        firstR &= !second;
                    }
                    else
                    {
                        firstL &= !second;
                    }

                    if(thetaDotMinus > thetaDot1)
                        thetaDotMinus = thetaDot1;

                    if(thetaDotPlus < thetaDot2)
                        thetaDotPlus = thetaDot2;
                }
                ++pedInterIt;
            }

            float alphaDotGoal = _alphaDotGoal;

            float thetaDotMin;
            float thetaDotMax;
            float firstMin;
            float firstMax;

            if(fabs(thetaDotPlus) < fabs(thetaDotMinus) )
            {
                thetaDotMin = thetaDotPlus;
                thetaDotMax = thetaDotMinus;
                firstMin = firstL;
                firstMax = firstR;
            }
            else
            {
                thetaDotMin = thetaDotMinus;
                thetaDotMax = thetaDotPlus;
                firstMin = firstR;
                firstMax = firstL;
            }

            if(!_waiting)
            {
                if( (thetaDotMin*alphaDotGoal > 0 && fabs(thetaDotMin) < fabs(alphaDotGoal)))
                {
                    thetaDot = alphaDotGoal;
                    first = firstMin;
                }
                else if( (thetaDotMax*alphaDotGoal > 0 && fabs(thetaDotMax) < fabs(alphaDotGoal)))
                {
                    thetaDot = alphaDotGoal;
                    first = firstMax;
                }
                else if(ttc > 1.0)
                {
                    float t1 = fabs(thetaDotMin - alphaDotGoal);
                    float t2 = fabs(thetaDotMax - alphaDotGoal);

                    if( (t1 < 0.01 && t2 < 0.01) || t1 <= t2 )
                    {
                        thetaDot =  thetaDotMin;
                        first = firstMin;
                    }
                    else
                    {
                        if(fabs(thetaDotMax) > 1.57)
                        {
                            thetaDot = thetaDotMax;
                            first = firstMax;
                        }
                        else
                        {
                            thetaDot = thetaDotMax;
                            first = false;
                        }
                    }
                }
                else
                {
                    /// don't use goal for close interaction
                    thetaDot =  thetaDotMin;
                    first = firstMin;
                }
                thetaDot = min(max(-1.57f, thetaDot), 1.57f);

                /// change of speed
                //                if(minttc < ttcTreshlod && (!first))
                //                {
                //                    speed = g_params[3] * ((1 - pow(2.718f , (float) (-(minttc*minttc)*0.4f) )));
                //                    if(speed < 0.6) speed = 0;
                //                }


                if(minttc < ttcTreshlod && (!first))
                {
                    float c = 1.3;
                    float gamma = 0.5;
                    float tau = 0.2;

                    float acc = c * (pedMinTTC->_pedPt->_velocity.longueur() - _velocity.longueur()) / pow((pedMinTTC->_distance), gamma);
                    speed = _velocity.longueur() + acc;

                    // 	if (!isnan(_position.longueur())) {
                    // 		if (isinf(acc)) {
                    // 			std::cout<< "acc\n";
                    // 			std::cout<< pedMinTTC->_distance<< std::endl;
                    // 		}
                    // 	}
                }



                /// stop and wait
                if(speed == 0.0 && _velocity.longueur() == 0.0 )
                {
                    //                    _waitTime = (mindtc < 0.0)? 0.1*(rand()%10) : 0.2 + (rand()%WAIT_TIME)*0.1;
                    _waitTime = 0.1+0.1f*(rand()%6);
                    _waiting = true;
                }
                else
                {
                    float tv = 0.5f;
                    float thetaDiff = max(-tv, min(tv, thetaDot - _thetaDotOld));
                    thetaDot = _thetaDotOld + thetaDiff;
                    _thetaDotOld = thetaDot;
                }
            }
            else
            {
                float t1 = fabs(thetaDotMin - alphaDotGoal);
                float t2 = fabs(thetaDotMax - alphaDotGoal);
                //
                thetaDot = thetaDotMin;
                thetaDot = ((t1 < 0.01 && t2 < 0.01) || t1 <= t2)? thetaDotMin : thetaDotMax;
                _waiting = false;
                speed = 0.001;
            }
            /// change of orientation
            Mat2 rmat; rmat.setRot(thetaDot);
            _velocityDesired = rmat * _orientation;

            _velocityDesired.normalize();
            _velocityDesired = _velocityDesired * speed;
        }
        else
        {
            _thetaDotOld = 0;
            _velocityDesired = _velocityComfort;
        }
    }
}

void
PedestrianV::updatePosition (float dt)
{
    _orientation = _orientationNew;
    _velocity = _velocityNew;
    _position = _positionNew;
}

void
PedestrianV::updateVelocity (float dt)
{
    _dt = dt;
    // _orientation = Vec2();
    _velocityDesired = Vec2();

    computeComfortVelocity();
    solveInteractions();

    Vec2 steering = _velocityDesired - _velocity;
    float speedDesired = _velocityDesired.longueur();

    if ( !( speedDesired < 0.05 && steering.longueur() < 0.1) )
    {
        Vec2 acceleration;
        float accMax = _accelerationMax;
        if(speedDesired == 0)
            accMax = 2*_accelerationMax;


        if (steering.longueur() > _dt * accMax)
        {
            steering.normalize();
            acceleration = steering * accMax * _dt;
        }
        else
        {
            acceleration = steering;
        }

        // update velocity
        _velocityNew = _velocity + acceleration ;

        // set orientation
        if(_velocityNew.longueur() > 0)
        {
            _orientationNew = _velocityNew;
            _orientationNew.normalize();
        }
    }
    else   // stop
    {
        _velocityNew = Vec2(0.0, 0.0);
    }

    _positionNew = _position + _velocityNew * dt;
    _velocityOld = _velocity;
}

void PedestrianV::setPosition(float s_x, float s_y)
{
    Vec2 v(s_x, s_y);
    setPosition(v);
}

void PedestrianV::setVelocity(float s_x, float s_y)
{
    Vec2 v(s_x, s_y);
    setVelocity(v);
}

void PedestrianV::setGoal(float s_x, float s_y)
{
    Vec2 v(s_x, s_y);
    setGoal(v, false);
}

float PedestrianV::getCenterx()
{
    return getPosition().x;
}

float PedestrianV::getCentery()
{
    return getPosition().y;
}

float PedestrianV::getCenterVelocityx()
{
    return getVelocity().x;
}

float PedestrianV::getCenterVelocityy()
{
    return getVelocity().y;
}

}
