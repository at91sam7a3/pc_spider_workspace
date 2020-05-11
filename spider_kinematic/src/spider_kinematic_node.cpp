#include <ros/ros.h>
#include <iostream>
#include <unistd.h>
//#include "Gyroscope.h"
//#include "Robot.h"
#include <signal.h>
#include "platform.h"

using namespace std;

void my_handler(int s){
    printf("Caught signal %d\n",s);
    //ServoManager::Down();// stop powering servos
    exit(1);

}

int main(int argc, char *argv[])
{
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    ros::init(argc,argv,"spider_kinematic");
    // ServoManager::Init();
    spider::Platform::GetInstance()->TestMovements();
    //spider::Platform::GetInstance()->MovementThread();


    return 0;
}
