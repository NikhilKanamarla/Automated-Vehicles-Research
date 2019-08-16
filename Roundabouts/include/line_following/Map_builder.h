#ifndef __MAPBUILDER
#define __MAPBUILDER

#include "Track_base.h"
#include "Line_track.h"
#include "Arc_track.h"
#include <string>
#include "Map_builder.h"
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

    void BuildMap();
};

#endif
