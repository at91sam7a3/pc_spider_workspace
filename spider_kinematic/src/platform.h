#ifndef PLATFORM_H
#define PLATFORM_H

#include "leg.h"

#include <memory>
#include <thread>
#include <vector>
#include "servomanager.h"
#include "geometry_msgs/Twist.h"

//This class manage all movements of robot
//It contain it`s own thread, and really autonomous
namespace spider {


enum MovementState{
    Sleeping=0,
    Idle,
    Going,
    Rotating
};

struct LegStatus{
    vec2f position;
    float height;
    static vec2f maxOffset;
    static float stepHeight;

};

class Platform
{
public:
    static Platform* GetInstance();
    //Immidiate stop
    void Stop();
    //Sit and power-off legs
    void Sleep();
    //Power-on legs
    void Wake();
    //coordinates
    vec2f GetCurrentPosition();
    //just movement
    void Turn(float degrees);

    void SetBodyHeight(float height);
    float GetBodyHeight();
    MovementState state_;
    void prepareToGo();
    void MovementThread();
    void TestMovements();
private:
    Platform();
    void movingEnd();
    void procedureGo();
    void procedureTurn();


    void MovementDelay();
    void velocityCallback(const geometry_msgs::Twist::ConstPtr msg);
    int getLegToRaise();
private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_velocity_sub;
    ServoManager m_servoManager;
    std::unique_ptr<std::thread> moving_thread_;
    vec2f currentCoordinates;
    int moveSpeed;
    float currentMovementSpeed;
    std::vector <Leg> legs_;
    double bodyHeight_;
    double directionAngle;
    double m_rotationSpeed;
    vec2f m_movementSpeed;
};
};
#endif // PLATFORM_H
