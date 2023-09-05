#include "../robot/main_app.h"
#include "unitree_Controller.h"

int main(int argc, char **argv)
{
    main_app(argc,argv,new unitree_Controller());
    return 0;
}