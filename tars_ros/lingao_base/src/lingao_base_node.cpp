
#include "base_driver.h"

int main(int argc, char** argv )
{
    ros::init(argc, argv, "lingao_base_driver");

    Base_Driver base_driver;
    base_driver.base_Loop();

    return 0;
}
