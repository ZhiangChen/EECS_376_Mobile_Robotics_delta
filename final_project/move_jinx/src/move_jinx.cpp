// Program to move jinx forward to the right table, reverse back and go to the left table goes here

// Written by Ananya, 4/14/16, 14:44

#include <iostream>
#include <ros/ros.h> //generic C++ stuff
using namespace std;

// This header incorporates all the necessary #include files and defines the class MoveJinx

#include <move_jinx/move_jinx.h>

MoveJinx::MoveJinx(ros::NodeHandle* nodehandle): nh_(*nodehandle)
{
}

bool MoveJinx::move2LT()
{
	return true;
}
bool MoveJinx::moveBack()
{return true;
}
bool MoveJinx::move2RT()
{return true;
}
