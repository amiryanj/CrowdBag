#ifndef PEDESTRIAN_VISION_H
#define PEDESTRIAN_VISION_H

#include "../craal-core/include/PDF.h"
#include "../craal-core/include/Pedestrian.h"

#include <list>
#include <vector>

#include "../simulations/mymath/include/seg2.h"
#include "../simulations/mymath/include/mat2.h"
#include "../simulations/mymath/include/vec2.h"
#include "../simulations/mymath/include/line.h"

namespace craal {

#define PED_INF 1e9

// namespace Vision
// {
class PedestrianV;

class PedestrianVInteraction
{
public:
    PedestrianVInteraction(void){};
    ~PedestrianVInteraction(void){};

    float   _alphaDot;
    float   _alpha;

    int     _collision;
    float  _distance;
    float  _distanceAtCollison;

    int     _id;
    Mat2    _matRot;

    PedestrianV* _pedPt;
    float  _personalAreaRadius;

    Vec2    _relativeAcceleration;
    Vec2    _relativePosition;
    Vec2    _relativeOwnVelocity;
    Vec2    _relativeVelocity;
    Vec2    _relativeComposedVelocity;
    float  _relativeDirectionAngle;

    float  _timeObservation;
    float  _ttc;
    float  _timeToCollision;

    Vec2    _velocity;
    Vec2    _orientation;
    bool    _visible;
};

/**
         * Vision model implemenation of @ref Pedestrian.
         */
class PedestrianV : public Pedestrian
{
public:
    PedestrianV();
    PedestrianV(Vec2 position, Vec2 goal, float speedComfort = 1.6,
                float speedMax = 2.0, float personalArea = 0.5);

    virtual ~PedestrianV();

    void restart();

    static void setParams()
    {
        addParam("a", -0.1, PDF(0, 1, PDF::NORMAL, 0.5, 0.2));
        addParam("b", 0.5, PDF(0.5, 1.5, PDF::NORMAL, 1, 0.2));
        addParam("c", 1, PDF(1, 2, PDF::NORMAL, 1.5, 0.2));
        addParam("Speed", 1.5, PDF(1, 2, PDF::NORMAL, 1.5, 0.5));
    };

    void   addObstacle(Seg2 o);
    void   addPath(Vec2 path1, bool inf);
    void   addPath(Vec2 path1, Vec2 path2, bool gateway, bool inf);
    virtual void   addPedestrianInteraction(Pedestrian *pedPt);
    bool   checkVisibility(const PedestrianV &ped);

    bool   getCircleCCW();
    bool   getCirclePath();
    bool   getInfPath();
    float getComfortSpeed();
    Vec2   getGoal();
    Vec2   getOrientation();
    Vec2   getOrientationNew();
    std::vector<Vec2> getPath();
    float getPersonalArea();
    Vec2   getPosition();
    Vec2   getPositionNew();

    bool   getVisible();
    Vec2   getVelocity();
    void   init(Vec2 position, Vec2 goal, float speedComfort = 1.6, float speedMax = 2.0,  float personalArea = 0.5);
    std::string info();
    virtual void   reset();
    virtual void addObstacle(float s_startx, float s_starty, float s_endx, float s_endy);
    void   setCircleCCW(bool b);
    void   setCirclePath(bool b);
    void   setInfPath(bool b);
    void   setComfortSpeed(float cs);
    void   setGoal(Vec2 g, bool dir);
    void   setPrint(bool p);
    void   setPath(const std::vector<Vec2>& path, bool inf);
    void   setPersonalArea(float r);
    void   setPosition(Vec2 p);
    void   setStartPosition(Vec2 p);
    void   setVelocity(Vec2 v);
    void   setVisible(PedestrianV *pedPt, bool v);
    virtual void   updatePosition(float dt);
    virtual void   updateVelocity(float dt);

    virtual void setPosition(float s_x, float s_y);
    virtual void setVelocity(float s_x, float s_y);
    virtual void setGoal(float s_x, float s_y);

    virtual float getCenterx();
    virtual float getCentery();
    virtual float getCenterVelocityx();
    virtual float getCenterVelocityy();

    std::list<PedestrianVInteraction> _pedestrianInteractions;
    PedestrianVInteraction  *pedInterToSolve;

    static std::vector<Seg2>	_sobstacles;
    // static std::vector<CircleObstacle>	_scobstacles;

    Vec2    _dirToGoal;
    //         float  _distToGoal;
    float   _alphaDotGoal;

private:

    void   computeComfortVelocity();
    void   computeGoalPoint();
    Vec2   computeNewVelocity(PedestrianVInteraction &pedInt,float t, Vec2 dir);

    void   computeIntersectionPoints(PedestrianVInteraction &pedInt);
    float computeCollisionProbability(PedestrianVInteraction &pedInt);
    std::vector<float> computeT(PedestrianVInteraction &pedInt, Line &tangent, Vec2 &tangentDir,
                                Vec2 &intPoint);
    float computeT(float tOpt, float tDev, float tSpd, float ttc);
    void   computeTangentPoints(PedestrianVInteraction &pedInt);
    void   computeTimeToContact(PedestrianVInteraction &pedIt);
    void   computeDirectionError(PedestrianVInteraction &pedInt);
    void   computeRelativeRepresentation(PedestrianVInteraction &pedInt);
    void   computeVelocityError(PedestrianVInteraction &pedInt);
    bool   computeSolutionToInteraction(PedestrianVInteraction &pedInt);

    void   init();
    bool   isACollision(PedestrianVInteraction &pedInt, Vec2 velocity, bool in);

    void   setDefaultPath();
    void   solveInteractions();
    void   solvePedestrianVInteractions();

    /// for deviation only alpha = 0,  for nearest point alpha = 0.5,
    /// for speed only alpha = 1

    float  _accelerationMax;
    bool    _infPath;
    bool	_direction;
    bool    _directionCircular;
    bool    _directionCircularCCW;
    //float  _distToGoal;
    //Vec2    _dirToGoal;
    float  _dt;
    Vec2    _goal;
    Vec2    _orientation;
    Vec2    _orientationNew;
    float  _personalAreaRadius;
    Vec2    _position;
    Vec2    _positionNew;
    bool    _print;
    int     _nwaypoints;
    int     _nextWaypoint;
    float  _speed;
    float  _speedComfort;
    float  _speedMax;
    Vec2    _velocity;
    Vec2    _velocityComfort;
    Vec2    _velocityDesired;
    Vec2    _velocityNew;
    Vec2    _velocityOld;

    std::vector<Vec2>	_path[2];
    std::vector<bool>	_pathGateway;
    Vec2	_pathPoint;
    int     _intPedCount;

    int    _start;
    float  _waitTime;
    bool   _waiting;
    bool  _recovering;
    int   _recoveryCount;

    float _thetaDotOld;

    float _p2;
    float _q2;

};
}
#endif
