#include "ros/ros.h"
#include "std_msgs/String.h"

#include "line_following/LineFollowing.h"
#include "line_following/CollisionCheck.h"

#include <vector>
#include <cmath>

#include <unordered_map>
#include <utility>
#include <set>

#include "Map_buildertest.h"
#include "Track_base.h"
#include <string>

#include <iostream>
#include <sstream>
#include <fstream>
#include <ctime>


using namespace std;

bool isRun = true;



  //Creating vectors which are being used from csv file
vector<double> ls;//Creating line segment number vector
vector<double> ld_x;//line direction vector in x direction
vector<double> ld_y;//line direction vector in y direction
vector<double> ll;//line length vector
vector<double> cx;
vector<double>cy;

  //Creating vectors which are being used from csv file
vector<double> as;//Creating arc segment number vector
vector<bool> ad; // creating the direction2 vector
vector <double> angle_s; // creating the  start angle vector 
vector <double> angle_e; // creating the  end angle vector 
vector<double> ox; // creating the center x vector
vector<double> oy; //creating teh cneter y vector 
vector<double> rad;//radius of an arc


MapBuilder theMap;
ifstream file("/home/themainframe/catkin_ws/src/line_following/src/line.csv");
ifstream file2("/home/themainframe/catkin_ws/src/line_following/src/Arcs.csv");
unordered_map<int,pair<double,double>>posTable;

void call()
{
    string Number, Index, Direction, Length, Originx, Originy, Cx, Cy;
    double Number_d, Direction_x,Direction_y, Length_d,Cx_d, Cy_d;
    string Number2, Index2,Direction2,StartAngle, EndAngle, Ox, Oy, Radius;
    bool Direction2_b;
    double Number_d2,StartAngle_d, EndAngle_d, Ox_d, Oy_d,Radius_d;

    for (int i=0;i<=92;i++)
    {
      
      getline(file,Number,',');
      stringstream geek1(Number); //geek is just a name for a stringstream variable 
      geek1>>Number_d;


      getline(file,Index,',');
      getline(file,Direction,',');

        if(Direction=="+Y"||Direction=="-Y"){
          Direction_x=0;
        if(Direction=="+Y"){
          Direction_y=-1;
        }else{
          Direction_y=1;  
        }
       }else if (Direction=="+X"){
          Direction_x=-1;
          Direction_y=0;
       }else {
          Direction_x=1;
          Direction_y=0;
         }

      getline(file,Length,',');
      stringstream geek2(Length);
      geek2>>Length_d;
      

      getline(file,Originx,',');
      getline(file,Originy,',');

      getline(file,Cx,',');
      stringstream geek3(Cx);
      geek3>>Cx_d;


      getline(file,Cy);
      stringstream geek4(Cy);
      geek4>>Cy_d;


        if(file.eof()==1)   // if we reach to the end of line, EOF becomes 1, then in the next line we check EOF to break the while loop if it is neccessary 
          {  file.close();
          break;};
      ls.push_back(Number_d);
      ld_x.push_back(Direction_x);
      ld_y.push_back(Direction_y);
      ll.push_back(Length_d);
      cx.push_back(Cx_d);
      cy.push_back(Cy_d);

      }
  
    for (int i=0;i<=116;i++)
  {
    getline(file2,Number2,',');
    stringstream geek1B(Number2);
    geek1B>>Number_d2;


    getline(file2,Index2,',');

    getline(file2,Ox,',');
    stringstream geek3B(Ox);
    geek3B>>Ox_d;

    getline(file2,Oy,',');
    stringstream geek4B(Oy);
    geek4B>>Oy_d;

    getline(file2,Radius,',');
    stringstream geek5B(Radius);
    geek5B>>Radius_d;


    getline(file2,StartAngle, ',');
    stringstream geek6B(StartAngle);
    geek6B >> StartAngle_d;

    getline(file2,EndAngle, ',');
    stringstream geek7B(EndAngle);
    geek7B >> EndAngle_d;

    getline(file2,Direction2);

    if(Direction2 =="CW"){
      Direction2_b = true;
    }
    else {
      Direction2_b = false;
    }


    if(file2.eof()==1)   // if we reach to the end of line, EOF becomes 1, then in the next line we check EOF to break the while loop if it is neccessary 
    {  file2.close();
    break;};
 
    as.push_back(Number_d2);
    ad.push_back(Direction2_b);
    angle_s.push_back(StartAngle_d);
    angle_e.push_back(EndAngle_d);
    rad.push_back(Radius_d);
    ox.push_back(Ox_d);
    oy.push_back(Oy_d);
  }
}
bool add(line_following::LineFollowing::Request  &req,
         line_following::LineFollowing::Response &res)
{
    float t1 = clock();
    int id = (int)req.id;
    double x = (double)req.x;
    double y = (double)req.y;
    ROS_INFO("Request: id=%d, x=%lf, y=%lf", id, x, y);

    if(id>=theMap.trackList.size())
    {
        res.res = 1;
        ROS_INFO("Failed to give result. Error code: 1 (track unregistered) and id = [%d]",id);
    }
    else if(!theMap.trackList[id] -> isWithinRange(x, y))
    {
        res.res = 2;
        ROS_INFO("Failed to give result. Error code: 2 (out of range)");
    }
    else
    {
        res.res = 0;
        theMap.trackList[id]->GetVector(x, y, res.dx, res.dy);
        double no = sqrt(res.dx*res.dx+res.dy*res.dy);
        res.dx = res.dx/no;
        res.dy = res.dy/no;

        ROS_INFO("Sending back response: [%lf, %lf]", res.dx, res.dy);
    }
    float t2 = clock();
    float tCalc=t2-t1;
    //ROS_INFO("Calculation time is : [%f]", tCalc/CLOCKS_PER_SEC);
    return true;
}

bool check(line_following::CollisionCheck::Request  &req,
         line_following::CollisionCheck::Response &res)
{
    int id = (int)req.id;
    double x = (double)req.x;
    double y = (double)req.y;
    double theta = (double)req.theta;
    int flag = (int)req.flag;
    if(flag)
    {
        posTable.erase(id);
        res.res = 1;
        res.ratio = 1;
        ROS_INFO("Erased robot: [%d]", id);
        return true;
    } 
    posTable[id] = make_pair(x,y);
    ROS_INFO("Check Request: id=%d", id);

    unordered_map<int,pair<double,double>>::iterator it = posTable.find(id);
    if(it==posTable.end())
    {
        res.res = 1;
        ROS_INFO("Failed to check collision. Error code: 1 (unregistered id)");
    }
    else
    {
        res.res = 0;
        double xEnd = cos(theta);
        double yEnd = sin(theta);
        LineTrack temp;//S94
        temp.SetWidth(0.1);//0.045);
        temp.SetOrigin(x,y);
        temp.SetDirection(xEnd,yEnd);
        temp.SetLength(0.27);

        bool isCrashed = false;
        for(auto p: posTable)
        {
            if(p.first!=id) 
            {
                double tx = (p.second).first;
                double ty = (p.second).second;
                if(temp.isWithinRange(tx,ty))
                {
                    isCrashed = true;
                    break;
                }
            }
        }
        if(isCrashed) res.ratio = 0;
        else res.ratio = 1;
        ROS_INFO("Sending back check response: [%lf]", res.ratio);
    }

    return true;
}

void cmdCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Heard command: [%s]", msg->data.c_str());
    if(strcmp(msg->data.c_str(),"exit")==0) isRun = false;
}

int main(int argc, char **argv)
{

    call();
    theMap.BuildMap(cx,cy,ld_x,ld_y,ll,ox,oy, rad,angle_s,angle_e,ad);
    ros::init(argc, argv, "lf_auto");

    ros::NodeHandle n;

    ros::MultiThreadedSpinner spinner(13); // Use 8 threads
    
    ros::ServiceServer service = n.advertiseService("lf_grad", add);
    ROS_INFO("Ready to calculate gradients based on the Automatic Mapbuildertest.cpp.");
    
    ros::Subscriber sub = n.subscribe("lf_cmd", 1000, cmdCallback);
    ROS_INFO("Welcome to line_following command center!");

    spinner.spin();

    return 0;
}
