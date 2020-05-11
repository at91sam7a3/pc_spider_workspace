#include "servomanager.h"
#include <iostream>
#include <map>
#include <functional>
#include <thread>
#include <chrono>
#include <iomanip>
#include "../../../devel/include/spider_messages/LegPower.h"
#include "../../../devel/include/spider_messages/Servo.h"

static int raw2degree(int in)
{
    float out=static_cast<float>(in)/757*180;
    return static_cast<int>(out);
}

//TODO set correct formula!
static int degree2raw(int in)
{
    float out=static_cast<float>(in)/180*757;
    return static_cast<int>(out);
}

ServoManager::ServoManager(ros::NodeHandle& nh)
    :m_nh(nh)
    ,power_pub(m_nh.advertise<spider_messages::LegPower>("servo_power", 10,true))
    ,servo_pub(m_nh.advertise<spider_messages::Servo>("servo_pos", 1000,true))
{
   ROS_INFO("Turning on power to servo");
   Init();
}

void ServoManager::Init()
{
    turnOnServoPower(true);
}

void ServoManager::Down()
{
    turnOnServoPower(false);
}

void ServoManager::setAngleF(int idx, double angle)
{
    if (idx>8) angle = 180.0 - angle;
    spider_messages::Servo s;
    s.Id = idx;
    s.Position = degree2raw(static_cast<int>(angle));
    servo_pub.publish(s);
    ros::spinOnce();
    ros::Duration(0.075).sleep();
}

void ServoManager::turnOnServoPower(bool state)
{
    if(state)
    {
        ROS_INFO("Turn Servo Power ON");
    }
    else
    {
        ROS_INFO("Turn Servo Power OFF");
    }
    spider_messages::LegPower p;
    p.PowerToLegs = state;
    power_pub.publish(p);
    ros::spinOnce();
}
