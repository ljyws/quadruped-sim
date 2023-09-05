#include "unitree_Controller.h"



unitree_Controller::unitree_Controller() : RobotController(){}

void unitree_Controller::initializeController()
{
    printf(" [unitree controller init] \n");
    //_gaitScheduler = new GaitScheduler(&userParameters,_controlParameters->controller_dt);//初始化一个步态i调度对象


    // _controlFSM = new ControlFSM();

}


void unitree_Controller::runController()
{
        printf(" [unitree controller runController] \n");
    // _gaitScheduler->loop();

    // _controlFSM->runFSM();
}