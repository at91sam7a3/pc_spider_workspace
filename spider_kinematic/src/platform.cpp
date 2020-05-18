#include "platform.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>


namespace spider {

static const double minimumDistanceStep=30; //TODO requires experiments

const double PI=3.141592654;

Platform *Platform::GetInstance()
{
    static Platform platform;
    return &platform;
}

//place legs in compact position for transportation
void Platform::Sleep()
{
    for(unsigned int i=0;i<6;++i)
    {
        //std::this_thread::sleep_for(std::chrono::milliseconds(150));
        ros::Duration(0.150).sleep();
        legs_[i].SetMotorAngle(0,180);

        ros::Duration(0.150).sleep();
        legs_[i].SetMotorAngle(1,0);

        ros::Duration(0.150).sleep();
        legs_[i].SetMotorAngle(2,0);
    }
}

void Platform::velocityCallback(const geometry_msgs::Twist::ConstPtr msg)
{
    m_movementSpeed = vec2f(msg->linear.x*10, msg->linear.y*10);
    m_rotationSpeed = msg->angular.z*2;
    std::string report = "Speed "+std::to_string(m_movementSpeed.x);
    ROS_INFO(report.c_str());
}

Platform::Platform()
    : state_(Idle)
    , m_nh()
    , m_velocity_sub(m_nh.subscribe("cmd_vel", 10, &Platform::velocityCallback, this))
    , m_servoManager(m_nh)
    , currentCoordinates(0,0)
    , currentMovementSpeed(0)
    , directionAngle(0.0) //lets say we looking at North
    , m_rotationSpeed(0.0f)
    , m_movementSpeed(0.0f, 0.0f)
{
    for (int i=0; i<6;++i)
    {
        Leg leg(m_servoManager);
        leg.SetLegIndex(i);
        legs_.push_back(leg);
    }
  //  prepareToGo();
}

void Platform::SetBodyHeight(float height)
{
    for (size_t i=0; i<6;++i)
    {
        legs_[i].bodyHeight_=height;
        legs_[i].RecalcAngles();
    }
    bodyHeight_=height;
}

float Platform::GetBodyHeight()
{
    return bodyHeight_;
}

void Platform::movingEnd()
{
    for(size_t i=0;i<6;++i){
        {
            legs_[i].MoveLegDown();
            legs_[i].RecalcAngles();
        }
    }
}

int Platform::getLegToRaise()
{
    int legToRaise = -1;
    double maxDist = 0;
    for(Leg& currentLeg : legs_)
    {
        double curDist = currentLeg.GetDistanceFromCenter();
        if(curDist > maxDist)
        {
            maxDist = curDist;
            legToRaise = currentLeg.GetLegIndex();
        }
    }
    if(maxDist > minimumDistanceStep)
    {
        legToRaise = -1;
    }
    return legToRaise;
}

void Platform::procedureGo()
{
    bool anyLegInAir = false; //main operation, if leg not in air - apply movement
    for(Leg& currentLeg : legs_)
    {
        if(currentLeg.leg_position != Leg::on_ground)
        {
            anyLegInAir = true;
            currentLeg.ProcessLegMovingInAir();
        }
        else //leg on a ground
        {
            //currentLeg.TurnLegWithGlobalCoord( m_rotationSpeed );
            currentLeg.LegAddOffsetInGlobal( m_movementSpeed.x, m_movementSpeed.y );
        }
    }
    if(!anyLegInAir)//all 6 legs on the ground, we check, do we need to raise any leg?
    {

        int legToRaise = getLegToRaise();

        if(legToRaise!=-1){ // if we have to raise any leg - do it

            vec2f newPoint(legs_[legToRaise].GetCenterVec());
            //vec2f tmpOffsetVec=m_movementSpeed * 0.25; //TODO - uncomment for possible optimization
            //newPoint=newPoint+tmpOffsetVec;
            legs_[legToRaise].MoveLegUp(newPoint);
        }

    }
    for(Leg& currentLeg : legs_)
    {
        currentLeg.RecalcAngles();//AMEN
    }
}


void Platform::prepareToGo()
{
    for(size_t i=0;i<6;++i){
        {
            if(!legs_[i].IsInCenter()){
                legs_[i].MoveLegUp();
                legs_[i].MoveLegToCenter();
                legs_[i].RecalcAngles();
                MovementDelay();
            }
            legs_[i].MoveLegDown();
            legs_[i].RecalcAngles();
            MovementDelay();
            MovementDelay();
        }
    }
}


void Platform::MovementThread()
{
    std::cout<<"Start movement thread"<<std::endl;
    ros::Rate r(10);
    prepareToGo();
    while(ros::ok())
    {
        procedureGo();
        //sleep if nothing to do
        ros::spinOnce();
        r.sleep();
       // ROS_INFO(".");
    }
}

void Platform::TestMovements()
{

    while(1){
        ros::Rate r(1);
        for (int i=0;i < 10 ; ++i)
        {
            m_servoManager.setAngleF(0,i*10);
            ros::spinOnce();
            r.sleep();
        }
     //   ros::Duration(5).sleep();

        ros::spin();
    }

}

void Platform::MovementDelay()
{
    ros::Duration(0.075).sleep();
}
}
