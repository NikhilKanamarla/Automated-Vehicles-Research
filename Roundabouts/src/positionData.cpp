#include "ros/ros.h"
#include "std_msgs/String.h"
#include <vector>
#include <chrono>
#include <node_handle.h>
using namespace std;
//This file globally manages positional data for all simulated vehicles, as long as information is sent to and requested from it.
//This means that simulations can interact with eachother, if that's what's wanted.
//Simply include a function to publish index,x,y data to the topic "veh_Pos"
//If you want to listen to the data, looked for the (Service?) "posTable"

//Author: Ben Remer
//Last Updated: 11/8/2018


//Global Variables
float pos[35][2]={};

int tableReturn(int index)
{
    return pos[id][0],pos[id][1]
}

void updateTable(int id, float x, float y)
{
    pos[id][0]=x;
    pos[id][1]=y;
}

int main(int argc, char **argv)
{
    
    cout<<"argc is "<<argc;

    ros::init(argc, argv, "posTable");

    ros::NodeHandle n;

    ros::MultiThreadedSpinner spinner(8); // Use 8 threads
    ros::Subscriber sub = n.subscribe("veh_Pos", 1000, updateTable);
    ROS_INFO("Welcome to UDSSC Vehicle Simulation Position Manager!");

    ros::ServiceServer service = n.advertiseService("posTable", tableReturn);
    ROS_INFO("Ready to provide positional data.");
    


    spinner.spin();

    return 0;
}