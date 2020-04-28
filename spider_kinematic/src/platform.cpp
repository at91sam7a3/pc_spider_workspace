#include "platform.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>


namespace spider {

static const double minimumDistanceStep=20; //TODO requires experiments

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
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
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
  if((msg->linear.x == 0.0) && (msg->linear.y == 0.0))
  {
      state_ = Idle;
  }
}

Platform::Platform()
    : state_(Idle)
    , m_nh()
    , m_velocity_sub(m_nh.subscribe("cmd_vel", 100, &Platform::velocityCallback, this))
    , m_servoManager(m_nh)
    , currentCoordinates(0,0)
    , moveSpeed(4)
    , normalizedMovementVector(1,0)
    , currentMovementSpeed(4)
    , distanceLeft_(0.0)
    , directionAngle(0.0) //lets say we looking at North
{
    for (int i=0; i<6;++i)
    {
        Leg leg(m_servoManager);
        leg.SetLegIndex(i);
        legs_.push_back(leg);
    }
    prepareToGo();
    moving_thread_.reset(new std::thread(&Platform::MovementThread,this));
}

void Platform::MoveForward(float distance)
{
    vec2f newMove;
    newMove.x = 1 * distance + currentCoordinates.x;
    newMove.y = 0 * distance + currentCoordinates.y;

    GoToCoordinates(newMove);
}



void Platform::GoToCoordinates(vec2f newCoord)
{
    std::cout<<"We at position "<<currentCoordinates<<std::endl;
    std::cout<<"Starting move to new position "<<newCoord.x<<","<<newCoord.y<<std::endl;
    vec2f newMove = newCoord - currentCoordinates;
    distanceLeft_ = newMove.size();
    if(distanceLeft_<0.001)
    {
        std::cout<<"Moving to current position not allowed, skipping"<<std::endl;
        normalizedMovementVector.x=0;
        normalizedMovementVector.y=0;
        return;
    }
    normalizedMovementVector.x=newMove.x / distanceLeft_;
    normalizedMovementVector.y=newMove.y / distanceLeft_;
    //normalizedMovementVector.rotate(); //if we look east and have to go north then locally we go west
    std::cout<<"changing state to going"<<std::endl;
    std::cout<<"movement vector "<<normalizedMovementVector<<std::endl;
    std::cout<<"distance "<<distanceLeft_<<std::endl;
    //prepareToGo();
    state_ = MovementState::Going;
}


void Platform::Turn(float degrees)
{
    std::cout <<"!!!Executing turn command"<<std::endl;
    desiredAngle_ = degrees;
    prepareToGo();
    state_ = MovementState::Rotating;
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

void Platform::procedureGo()
{
    bool anyLegInAir = false; //so, only one leg allowed to be in air at once
    for(Leg& currentLeg : legs_)
    {
        if(currentLeg.leg_position != Leg::on_ground)
        {
            anyLegInAir = true;
            currentLeg.ProcessLegMovingInAir();
        }
        else //leg on a ground
        {
            if(distanceLeft_>currentMovementSpeed) //make a full step
            {
                currentLeg.LegAddOffsetInGlobal( normalizedMovementVector.x * currentMovementSpeed
                                                 ,normalizedMovementVector.y * currentMovementSpeed );
            }
            else {
                currentLeg.LegAddOffsetInGlobal(normalizedMovementVector.x*distanceLeft_,normalizedMovementVector.y*distanceLeft_);
            }
        }
    }
    if(distanceLeft_>currentMovementSpeed) //make a full step
    {
        distanceLeft_-=currentMovementSpeed;
        currentCoordinates.x+=normalizedMovementVector.x*currentMovementSpeed;
        currentCoordinates.y+=normalizedMovementVector.y*currentMovementSpeed;
    }
    else {
        currentCoordinates.x+=normalizedMovementVector.x*distanceLeft_;
        currentCoordinates.y+=normalizedMovementVector.y*distanceLeft_;
        std::cout<<"We came to position "<<currentCoordinates<<std::endl;
        distanceLeft_=0;
        state_ = MovementState::Idle;
        movingEnd();
        return;
    }

    std::cout<<"distance left ="<<distanceLeft_<<std::endl;;
    if(!anyLegInAir)//all 6 legs on the ground
    {
        double maxDist=0;
        int legToRaise=-1;
        for(Leg& currentLeg : legs_)
        {
            double curDist = currentLeg.GetDistanceFromCenter();
            if(curDist>maxDist)
            {
                maxDist=curDist;
                legToRaise=currentLeg.GetLegIndex();
            }
        }
        if(maxDist>=minimumDistanceStep){ //Dont make steps if it not really needed
            std::cout<<"Move up leg # "<<legToRaise<<std::endl;
            if(legToRaise!=-1){ // I think this check not needed, but let it be

                vec2f newPoint(legs_[legToRaise].GetCenterVec());
                std::cout<<"Center is "<<newPoint<<std::endl;
                vec2f tmpOffsetVec=normalizedMovementVector*(maxDist/4); //was /2
                newPoint=newPoint+tmpOffsetVec;
                std::cout<<"new point "<<newPoint<<std::endl;
                legs_[legToRaise].MoveLegUp(newPoint);
            }
        }
    }
    for(Leg& currentLeg : legs_)
    {
        currentLeg.RecalcAngles();//AMEN
    }
}

void Platform::procedureTurn()
{
    double rotationSpeed=1;
    bool anyLegInAir = false; //so, only one leg allowed to be in air at once
    double rot=0;
    //here we check where to turn
    if(directionAngle > desiredAngle_)
        rot=-rotationSpeed;
    else
        rot=rotationSpeed;


    std::cout<<"Procedure turn____________________"<<std::endl;
    std::cout<<"angle "<<directionAngle<<std::endl;
    std::cout<<"desiredAngle_ angle "<<desiredAngle_<<std::endl;
    std::cout<<"rot"<<rot<<std::endl;

    for(Leg& currentLeg : legs_)
    {
        if(currentLeg.leg_position != Leg::on_ground)
        {
            anyLegInAir = true;
            currentLeg.ProcessLegMovingInAir();
        }
        else //leg on a ground
        {
            currentLeg.TurnLegWithGlobalCoord(rot);
            currentLeg.RecalcAngles();
        }
    }
    directionAngle+=rot;

    if(fabs(directionAngle-desiredAngle_)<rotationSpeed)
    {
        state_ = MovementState::Idle;
        movingEnd();
        return;
    }


    if(!anyLegInAir)//all 6 legs on the ground
    {
        double maxDist=0;
        int legToRaise=-1;
        for(Leg& currentLeg : legs_)
        {
            double curDist = (currentLeg.GetDistanceFromCenter());
            std::cout<<"cur dist # "<<curDist<<std::endl;
            if(curDist>maxDist)
            {
                maxDist=curDist;
                legToRaise=currentLeg.GetLegIndex();
            }
        }
        std::cout<<"max dist # "<<legToRaise<<std::endl;
        if(maxDist>=45){ //Dont make steps if it not really needed
            std::cout<<"Move up leg # "<<legToRaise<<std::endl;
            if(legToRaise!=-1){ // I think this check not needed, but let it be

                vec2f newPoint(legs_[legToRaise].GetCenterVec());
                // std::cout<<"Center is "<<newPoint.x<<" , "<<newPoint.y<<std::endl;
                legs_[legToRaise].MoveLegUp(newPoint);
            }
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
    do
    {
        //  std::cout << " Move: sIterating move thread, state="<<state_ <<std::flush<<std::endl;
        //So, here we are
        //We need to think, have we make some movement?
        switch(state_)
        {
        case spider::Sleeping:
            //Clear that it`s power off, and nothing to do here
            break;
        case spider::Idle:

            break;
        case spider::Going:
            procedureGo();
          //  TelemetryManager::GetInstance()->setValue("X Coord",currentCoordinates.x);
          //  TelemetryManager::GetInstance()->setValue("Y Coord",currentCoordinates.y);
            break;
        case spider::Rotating:
            //TODO not implementer yet
            procedureTurn();
          //  TelemetryManager::GetInstance()->setValue("Direction",directionAngle);
            break;
        default:
            break;
        }
        //sleep if nothing to do
        MovementDelay();
    }
    while(true);
}

void Platform::MovementDelay()
{
    //std::this_thread::sleep_for(std::chrono::milliseconds(75));
    ros::Duration(0.075).sleep();
}
}
