#include "Merge_builder.h"
#include "Control_zone.h"

#include <queue>
#include <ctime>
#include <tuple>

using namespace std;

MergeBuilder::MergeBuilder()
{
    zoneList.clear();
}

MergeBuilder::~MergeBuilder()
{
    while(!zoneList.empty())
    {
        delete zoneList[zoneList.size()-1];
        zoneList.pop_back();
    }
}

void MergeBuilder::BuildMerge()
{
    ControlZone* z = new ControlZone(0);//This is For Mergsim.py
    z->init_time = time(0);
    z->entrences.push_back(EntrenceInfo{1.3,0.4,0.36});//these are for the merging Scenario 0.35
    z->entrences.push_back(EntrenceInfo{1.3,0.4,0.36});//these are for the merging scenario 
    zoneList.push_back(z);

    ControlZone* z1 = new ControlZone(0);//Intersim.py
    z1->init_time = time(0);
    z1->entrences.push_back(EntrenceInfo{1.5,0.5,0.45}); //L,S,delta for the intersection problem 1.5 .48 .35
    z1->entrences.push_back(EntrenceInfo{1.5,0.5,0.45}); //L,S,delta for the intersection problem

    zoneList.push_back(z1);

    ControlZone* z2 = new ControlZone(0);//Intersim.py
    z2->init_time = time(0);
    z2->entrences.push_back(EntrenceInfo{1.5,0.5,0.45}); //L,S,delta for the intersection problem
    z2->entrences.push_back(EntrenceInfo{1.5,0.5,0.45}); //L,S,delta for the intersection problem
    zoneList.push_back(z2);

    ControlZone* z3 = new ControlZone(0);//Test.py
    z3->init_time = time(0);
    z3->entrences.push_back(EntrenceInfo{3.0,0.6,0.55}); //L,S,delta 
    z3->entrences.push_back(EntrenceInfo{3.0,0.6,0.55}); //L,S,delta 
    zoneList.push_back(z3);

    //ControlZone* z4= new ControlZone(0);//Test.py
    //#z4->init_time = time(0);
    //z4->entrences.push_back(EntrenceInfo{1.5,0.5,0.45}); //L,S,delta (eid or control type the third number which you put in path.py for defining the control zone)
    //z4->entrences.push_back(EntrenceInfo{1.5,0.5,0.45}); //L,S,delta 
    //z4->entrences.push_back(EntrenceInfo{1.5,0.5,0.45}); //L,S,delta 
    //zoneList.push_back(z4);
    # Incresed saftey distance
    ControlZone* z4 = new ControlZone(0);//This is For NKroundabout/Mergsim.py 
    z4->init_time = time(0);
    z4->entrences.push_back(EntrenceInfo{3.0,0.6,0.95});//these are for the roundabout
    z4->entrences.push_back(EntrenceInfo{3.0,0.6,0.95});//these are for the roundabout 
    zoneList.push_back(z4);
}
