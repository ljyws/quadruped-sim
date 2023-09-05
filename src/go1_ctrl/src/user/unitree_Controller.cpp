#include "unitree_Controller.h"



unitree_Controller::unitree_Controller() : RobotController(){}

void unitree_Controller::initializeController()
{
    //_gaitScheduler = new GaitScheduler(&userParameters,_controlParameters->controller_dt);//初始化一个步态i调度对象


    // _controlFSM = new ControlFSM();


}


void unitree_Controller::runController()
{
    // _gaitScheduler->loop();

    // _controlFSM->runFSM();
}