#ifndef __MAPBUILDER
#define __MAPBUILDER

#include "Track_base.h"
#include "Line_track.h"
#include "Arc_track.h"
#include <string>
#include "Map_buildertest.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>


using namespace std;

class MapBuilder
{
public:
    MapBuilder();
    ~MapBuilder();

    
    vector<TrackBase*> trackList;

    void BuildMap(vector<double> cx,vector<double> cy,vector<double> ld_x,vector<double> ld_y,vector<double> ll,vector<double> ox,vector<double> oy,vector<double> rad,vector<double> angle_s,vector<double> angle_e,vector<bool> ad);
};

#endif
