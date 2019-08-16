
//This is a Test in CPP Folder

#include "Map_buildertest.h"
#include "Track_base.h"
#include "Line_track.h"
#include "Arc_track.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>


using namespace std;

MapBuilder::MapBuilder()
{
  trackList.clear();  

}

MapBuilder::~MapBuilder()
{
    while(!trackList.empty())
    {
        delete trackList[trackList.size()-1];
        trackList.pop_back();
    }
}

void MapBuilder::BuildMap(vector<double> cx,vector<double> cy,vector<double> ld_x,vector<double> ld_y,vector<double> ll,vector<double> ox,vector<double> oy,vector<double> rad,vector<double> angle_s,vector<double> angle_e,vector<bool> ad)
{ 

  LineTrack* lt;
  for(int i=0; i<=92; i++){
    lt=new LineTrack;
    lt->SetWidth(0.25);//0.2286
    lt->SetOrigin(cx[i],cy[i]);
    lt->SetDirection(ld_x[i],ld_y[i]);
    lt->SetLength(ll[i]);
    lt->SetParameter(2);
    trackList.push_back(lt); 
  }
  
   ArcTrack* lt2;
  for(int i=0; i<=116; i++){
    lt2=new ArcTrack;
    lt2->SetCenter(ox[i],oy[i]);
    lt2->SetRadius(rad[i]);
    lt2->SetAngleRange(angle_s[i],angle_e[i]);
    lt2->SetWidth(0.2286);
    lt2->SetDirection(ad[i]); //false is ccw
    lt2->SetRangeDirection(false);
    lt2->SetParameter(.2);
    trackList.push_back(lt2);}
    cout<<"HERE IS "<<cx[74]<<endl;
    cout<<"HERE IS "<<cy[74]<<endl;
    cout<<"HERE IS "<<ld_x[74]<<endl;
    cout<<"HERE IS "<<ld_y[74]<<endl;
    cout<<"HERE IS "<<ll[74]<<endl;

 /*
    LineTrack* lt1 = new LineTrack;//S68
    lt1->SetWidth(0.2286);
    lt1->SetOrigin(cx[0],cy[0]);
    lt1->SetDirection(ld_x[0],ld_y[0]);
    lt1->SetLength(ll[0]);
    lt1->SetParameter(2);
    trackList.push_back(lt1);
*/

}
