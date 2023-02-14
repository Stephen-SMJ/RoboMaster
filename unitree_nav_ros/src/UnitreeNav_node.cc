/*
 * Copyright 2022
 *
 * Author : Jingrao.Zhang@geely.com
 * Date : 2022-03-22
 * 
 * Description : Navigation for A1 quadruped robot. 
 * unitree_nav_node.
 */

#include  "unitree_nav_ros/libUnitreeNav.h"

int main(int argc,  char ** argv) 
{
    ros::init(argc,  argv,  "unitree_nav_node");

    UnitreeNav MyUnitreeNav;
    MyUnitreeNav.Manager();

    return 0;
}
