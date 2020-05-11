#ifndef SERVOMANAGER_H
#define SERVOMANAGER_H

#include <ros/ros.h>

class ServoManager
{
public:
    ServoManager(ros::NodeHandle& m_nh);
    ~ServoManager() = default;
    void Init();
    void Down();
    void setAngleF(int idx, double angle);
    void turnOnServoPower(bool on);
private:
    ros::NodeHandle& m_nh;
    ros::Publisher power_pub;
    ros::Publisher servo_pub;
};

#endif // SERVOMANAGER_H
