#include "LSM6DSO_Task.hpp"


LSM6DSO_Handle IMU;

void LSM6DSO_Task(void *argument)
{
    while (1)
    {
        cprintf(&huart3, "ok\n");
        //do nothing
    }
    
}