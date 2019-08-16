#include "Map_builder.h"
#include "Track_base.h"
#include "Line_track.h"
#include "Arc_track.h"

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

void MapBuilder::BuildMap()
{ 
    //source 0
    LineTrack* lt1 = new LineTrack;//S94
    lt1->SetWidth(0.2286);
    lt1->SetOrigin(2.4352,1.2192-0.02);
    lt1->SetDirection(-1,0);
    lt1->SetLength(3.3096+0.5);
    lt1->SetParameter(2);

    trackList.push_back(lt1);

    //source 1
    ArcTrack* lt2 = new ArcTrack;//A106
    lt2->SetCenter(-0.762,0.-2.6924);
    lt2->SetRadius(3.9115);
    lt2->SetAngleRange(-1.5708,-1.20428);
    lt2->SetWidth(0.2286);
    lt2->SetDirection(false); //false is ccw
    lt2->SetRangeDirection(false);
    lt2->SetParameter(.2);

    trackList.push_back(lt2);

    //source 2
    ArcTrack* lt3 = new ArcTrack;//A105-A101-A100
    lt3->SetCenter(-2.032 -.01,0.3556+0.01); //(-2.032 -.01,0.3556+.2)
    lt3->SetRadius(0.6096-.01);
    lt3->SetAngleRange(-1.172861,3.0102);
    lt3->SetWidth(0.2286);
    lt3->SetDirection(false);
    lt3->SetRangeDirection(false);
    lt3->SetParameter(.2);

    trackList.push_back(lt3);

    //source 3
    ArcTrack* lt4 = new ArcTrack;//A99
    lt4->SetCenter(-0.8744,0.2042);
    lt4->SetRadius(0.5574);
    lt4->SetAngleRange(-3.1414,-1.5708);// I am not sure about the order of this !(Behdad)
    lt4->SetWidth(0.2286);
    lt4->SetDirection(true);
    lt4->SetRangeDirection(false);
    lt4->SetParameter(.2);

    trackList.push_back(lt4);

    //source 4
    LineTrack* lt5 = new LineTrack;//S95-S91-S89 (just had to increase length to combine)
    lt5->SetWidth(0.2286);
    lt5->SetOrigin(-0.8744,0.7616);
    lt5->SetDirection(1,0);
    lt5->SetLength(2.9945);
    lt5->SetParameter(2);
    
    trackList.push_back(lt5);

    //source 5
    ArcTrack* lt6 = new ArcTrack;//A92
    lt6->SetCenter(1.0676,0.1286);
    lt6->SetRadius(0.6877);
    lt6->SetAngleRange(-1.6650,-2.7541);
    lt6->SetWidth(0.2286);
    lt6->SetDirection(true);
    lt6->SetRangeDirection(false);
    lt6->SetParameter(0.2);

    trackList.push_back(lt6);    

    //source 6
    ArcTrack* lt7 = new ArcTrack;//A81
    lt7->SetCenter(2.2352,0.6096);
    lt7->SetRadius(0.59);//0.6096
    lt7->SetAngleRange(0.3194,1.1694); //origially -4.5,-1.5708
    lt7->SetWidth(0.2286);
    lt7->SetDirection(false);
    lt7->SetRangeDirection(false);
    lt7->SetParameter(0.2);

    trackList.push_back(lt7); 

 //Road 2
    //source 7
    LineTrack* lt8 = new LineTrack;//S61
    lt8->SetWidth(0.2286);
    lt8->SetOrigin(2.0066,-0.98);
    lt8->SetDirection(-1,0);
    lt8->SetLength(1.6256);
    lt8->SetParameter(2);
    
    trackList.push_back(lt8);

    //source 8
    ArcTrack* lt9 = new ArcTrack;//A58
    lt9->SetCenter(0.34+0.04,-0.8);
    lt9->SetRadius(0.1224);
    lt9->SetAngleRange(1.5708,-3.0);
    lt9->SetWidth(0.2286);
    lt9->SetDirection(true);
    lt9->SetRangeDirection(false);
    lt9->SetParameter(0.9); //note this was 0.9 before, but all the others are 0.2 so I changed it

    trackList.push_back(lt9); 

    //source 9
    LineTrack* lt10 = new LineTrack;//S80
    lt10->SetWidth(0.2286);
    lt10->SetOrigin(0.2150,-0.8082-3);
    lt10->SetDirection(0,1);
    lt10->SetLength(1.343+3);
    lt10->SetParameter(2);
    
    trackList.push_back(lt10);

    //source 10
    ArcTrack* lt11 = new ArcTrack;//A94
    lt11->SetCenter(0.6858,0.3048);
    lt11->SetRadius(0.4572);
    lt11->SetAngleRange(3.1414,-2.7244);
    lt11->SetWidth(0.2286);
    lt11->SetDirection(true);
    lt11->SetRangeDirection(false);
    lt11->SetParameter(0.2);

    trackList.push_back(lt11); 

    //source
    //S95-S91-S89 (just had to increase length to combine)

    //source
    //A92  

    //source
    //A87-84-81

    //source 11
    ArcTrack* lt12 = new ArcTrack;//A86
    lt12->SetCenter(1.8826-0.04,-0.2032);
    lt12->SetRadius(0.2764); //0.2763
    lt12->SetAngleRange(-1.16,0.0);
    lt12->SetWidth(0.2286);
    lt12->SetDirection(true); //false is ccw
    lt12->SetRangeDirection(false);
    lt12->SetParameter(0.2);

    trackList.push_back(lt12); 

    //source 12
    LineTrack* lt13 = new LineTrack;//S88
    lt13->SetWidth(0.2286);
    lt13->SetOrigin(2.159,.1032);
    lt13->SetDirection(0,-1);
    lt13->SetLength(0.935);
    lt13->SetParameter(2);
    
    trackList.push_back(lt13);

    //source 13
    ArcTrack* lt14 = new ArcTrack;//A53
    lt14->SetCenter(2.0066,-0.8382+0.02);
    lt14->SetRadius(0.1524);
    lt14->SetAngleRange(0.17,1.2508);
    lt14->SetWidth(0.2286);
    lt14->SetDirection(true);
    lt14->SetRangeDirection(false);
    lt14->SetParameter(0.2);

    trackList.push_back(lt14); 

    //source 14
    ArcTrack* lt15 = new ArcTrack;//A87
    lt15->SetCenter(2.2352-0.02,0.6096);
    lt15->SetRadius(0.59);
    lt15->SetAngleRange(1.1694,2.5656); //origially -4.5,-1.5708
    lt15->SetWidth(0.2286);
    lt15->SetDirection(false);
    lt15->SetRangeDirection(false);
    lt15->SetParameter(0.2);

    trackList.push_back(lt15);
    
    //source 15
    ArcTrack* lt16 = new ArcTrack;//A84
    lt16->SetCenter(2.2352-0.02,0.6096);
    lt16->SetRadius(0.59);
    lt16->SetAngleRange(2.5656,-1.5708); //origially -4.5,-1.5708
    lt16->SetWidth(0.2286);
    lt16->SetDirection(false);
    lt16->SetRangeDirection(false);
    lt16->SetParameter(0.2);

    trackList.push_back(lt16); 

    //source 16 // Test
    LineTrack* lt17 = new LineTrack;//
    lt17->SetWidth(0.2286);
    lt17->SetOrigin(1,-3.524);
    lt17->SetDirection(-1,0);
    lt17->SetLength(3);
    lt17->SetParameter(2);
    
    trackList.push_back(lt17);

    //source 17 // Test
    LineTrack* lt18 = new LineTrack;//
    lt18->SetWidth(0.2286);
    lt18->SetOrigin(-2,-3.524);
    lt18->SetDirection(1,0);
    lt18->SetLength(2);
    lt18->SetParameter(2);
    
    trackList.push_back(lt18);

    //////////////////left to right intersection Path1
    //source 18 
    LineTrack* lt19 = new LineTrack;//// S23
    lt19->SetWidth(0.2286);
    lt19->SetOrigin(2.159,-3.302);// 
    lt19->SetDirection(-1,0);
    lt19->SetLength(1.2192);
    lt19->SetParameter(2);
    
    trackList.push_back(lt19);


   //source 19 
    LineTrack* lt20 = new LineTrack;//// S22
    lt20->SetWidth(0.2286);
    lt20->SetOrigin(0.9398,-3.302);
    lt20->SetDirection(-1,0);
    lt20->SetLength(.7112);
    lt20->SetParameter(2);
    
    trackList.push_back(lt20);

   //source 20 
    LineTrack* lt21 = new LineTrack;//// S19 MERGING ZONE 
    lt21->SetWidth(0.2286);
    lt21->SetOrigin(0.2286,-3.302);
    lt21->SetDirection(-1,0);
    lt21->SetLength(.4572);
    lt21->SetParameter(2);
    
    trackList.push_back(lt21);

   //source 21 
    LineTrack* lt22 = new LineTrack;//// S14
    lt22->SetWidth(0.2286);
    lt22->SetOrigin(-0.2286,-3.302);
    lt22->SetDirection(-1,0);
    lt22->SetLength(1.5748);
    lt22->SetParameter(2);
    
    trackList.push_back(lt22);

/////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
///path2

   //source 22 
    LineTrack* lt23 = new LineTrack;//// S68
    lt23->SetWidth(0.2286);
    lt23->SetOrigin(-1.5383-1,-1.2972);//1.524 is subtracted from y to move in the center of the lower part of map 
    lt23->SetDirection(1,0);
    lt23->SetLength(1.1574+1);
    lt23->SetParameter(2);
    
    trackList.push_back(lt23);
    
    //source 23
    ArcTrack* lt24 = new ArcTrack;//A64
    lt24->SetCenter(-0.381,-1.6);
    lt24->SetRadius(0.3048);
    lt24->SetAngleRange(1.5708,0); 
    lt24->SetWidth(0.2286);
    lt24->SetDirection(true);
    lt24->SetRangeDirection(false);
    lt24->SetParameter(0.2);

     trackList.push_back(lt24); 

    //source 24 
    LineTrack* lt25 = new LineTrack;//// S47
    lt25->SetWidth(0.2286);
    lt25->SetOrigin(-0.0762,-1.6);//1.524 is subtracted from y to move in the center of the lower part of map 
    lt25->SetDirection(0,-1);
    lt25->SetLength(1.0049);
    lt25->SetParameter(2);
    
    trackList.push_back(lt25);

        //source 25
    LineTrack* lt26 = new LineTrack;//// S31
    lt26->SetWidth(0.2286);
    lt26->SetOrigin(-0.0762,-2.6049);//1.524 is subtracted from y to move in the center of the lower part of map 
    lt26->SetDirection(0,-1);
    lt26->SetLength(0.5444);
    lt26->SetParameter(2);
    
    trackList.push_back(lt26);

        //source 26
    LineTrack* lt27 = new LineTrack;//// S17 //MERGING ZONE
    lt27->SetWidth(0.2286);
    lt27->SetOrigin(-0.0762,-3.1496);//1.524 is subtracted from y to move in the center of the lower part of map 
    lt27->SetDirection(0,-1);
    lt27->SetLength(0.5);//0.4572
    lt27->SetParameter(2);
    
    trackList.push_back(lt27);

        //source 27
    LineTrack* lt28 = new LineTrack;//// S15
    lt28->SetWidth(0.2286);
    lt28->SetOrigin(-0.0762,-3.6068);//1.524 is subtracted from y to move in the center of the lower part of map 
    lt28->SetDirection(0,-1);
    lt28->SetLength(0.381);
    lt28->SetParameter(2);
    
    trackList.push_back(lt28);

        //source 28
    ArcTrack* lt29 = new ArcTrack;//A10
    lt29->SetCenter(-0.2296-0.10,-3.9878+0.10);
    lt29->SetRadius(0.1524+0.1);
    lt29->SetAngleRange(0,-1.4); 
    lt29->SetWidth(0.2286);
    lt29->SetDirection(true);
    lt29->SetRangeDirection(false);
    lt29->SetParameter(0.2);

     trackList.push_back(lt29); 

             //source 29
    LineTrack* lt30 = new LineTrack;//// S6
    lt30->SetWidth(0.2286);
    lt30->SetOrigin(-0.2286,-4.14);//1.524 is subtracted from y to move in the center of the lower part of map 
    lt30->SetDirection(-1,0);
    lt30->SetLength(1.5748);
    lt30->SetParameter(2);
    
    trackList.push_back(lt30);

        //source 30
    ArcTrack* lt31 = new ArcTrack;//A9
    lt31->SetCenter(-1.8,-3.98);
    lt31->SetRadius(0.1524);
    lt31->SetAngleRange(-1.5708,3.1416); 
    lt31->SetWidth(0.2286);
    lt31->SetDirection(true);
    lt31->SetRangeDirection(false);
    lt31->SetParameter(0.2);

     trackList.push_back(lt31); 

             //source 31 /// i should combine S10-S11-S34 S40 S45
    LineTrack* lt32 = new LineTrack;//// S10
    lt32->SetWidth(0.2286);
    lt32->SetOrigin(-1.97,-3.9878);//-1.9558   1.524 is subtracted from y to move in the center of the lower part of map 
    lt32->SetDirection(0,1);
    lt32->SetLength(2.54);//0.381+0.6858+0.4572+0.5588+.4572=2.54
    lt32->SetParameter(2);
    
    trackList.push_back(lt32);

            //source 32
    ArcTrack* lt33 = new ArcTrack;//A72
    lt33->SetCenter(-1.79+0.1,-1.4478-0.1);
    lt33->SetRadius(0.1524+0.1);
    lt33->SetAngleRange(-3.1416,+1.5708); 
    lt33->SetWidth(0.2286);
    lt33->SetDirection(true);
    lt33->SetRangeDirection(false);
    lt33->SetParameter(0.2);

     trackList.push_back(lt33); 

            //source 33
    LineTrack* lt34 = new LineTrack;//// S74
    lt34->SetWidth(0.2286);
    lt34->SetOrigin(-1.8034,-1.2954);//1.524 is subtracted from y to move in the center of the lower part of map 
    lt34->SetDirection(1,0);
    lt34->SetLength(0.265);
    lt34->SetParameter(2);
    
    trackList.push_back(lt34);

///////////////////////////S29 
    //source 34 
    LineTrack* lt35 = new LineTrack;//// S29+S51 path1 
    lt35->SetWidth(0.2286);
    lt35->SetOrigin(2.3114,-3.302+0.5444+1.0049+1);//=-1.7527  for one path but it was chosen bellow this 
    lt35->SetDirection(0,-1);
    lt35->SetLength(0.5444+1.0049+1);//1.5493
    lt35->SetParameter(2);
    
    trackList.push_back(lt35);

        //source 35
    LineTrack* lt36 = new LineTrack;//// S62+S58
    lt36->SetWidth(0.2286);
    lt36->SetOrigin(-0.3809,-1.2972);// 
    lt36->SetDirection(1,0);
    lt36->SetLength(1.6256+0.762);
    lt36->SetParameter(2);
    
    trackList.push_back(lt36);

        //source 36
    ArcTrack* lt37 = new ArcTrack;//A51  down left path1
    lt37->SetCenter(2.0,-1.6);
    lt37->SetRadius(0.3048);
    lt37->SetAngleRange(1.5708,0); 
    lt37->SetWidth(0.2286);
    lt37->SetDirection(true);
    lt37->SetRangeDirection(false);
    lt37->SetParameter(0.2);

     trackList.push_back(lt37); 

        //source 37
    ArcTrack* lt38 = new ArcTrack;//A14 road
    lt38->SetCenter(2.159,-1.6256-1.524);
    lt38->SetRadius(0.1524);
    lt38->SetAngleRange(0,-1.5708); 
    lt38->SetWidth(0.2286);
    lt38->SetDirection(true);
    lt38->SetRangeDirection(false);
    lt38->SetParameter(0.2);
     trackList.push_back(lt38); 

        //source 38
    ArcTrack* lt39 = new ArcTrack;//A23
    lt39->SetCenter(-1.83,-1.6256-1.526);
    lt39->SetRadius(0.1524);
    lt39->SetAngleRange(-1.5708,-3); 
    lt39->SetWidth(0.2286);
    lt39->SetDirection(true);
    lt39->SetRangeDirection(false);
    lt39->SetParameter(0.2);
     trackList.push_back(lt39);

        //Source 39
    ArcTrack* lt40 = new ArcTrack;//A14 road
    lt40->SetCenter(2.159,-1.6256-1.524);
    lt40->SetRadius(0.1524);
    lt40->SetAngleRange(0,-1.5708); 
    lt40->SetWidth(0.2286); //same for every 
    lt40->SetDirection(true);
    lt40->SetRangeDirection(false);
    lt40->SetParameter(0.2);
     trackList.push_back(lt40); 

        //source 40
    LineTrack* lt41 = new LineTrack;//// S84
    lt41->SetWidth(0.2286);
    lt41->SetOrigin(-0.3809,-1.2972);// 
    lt41->SetDirection(1,0);
    lt41->SetLength(1.6256+0.762);
    lt41->SetParameter(2);
    
    trackList.push_back(lt41);

    
        //source 41
    LineTrack* lt42 = new LineTrack;//// S60
    lt42->SetWidth(0.2286);
    lt42->SetOrigin(2.01,-1.15);// 
    lt42->SetDirection(-1,0);
    lt42->SetLength(1.63);
    lt42->SetParameter(2);
    
    trackList.push_back(lt42);   

        //source 42
    LineTrack* lt43 = new LineTrack;//// S71
    lt43->SetWidth(0.2286);
    lt43->SetOrigin(0.38,-1.15);// 
    lt43->SetDirection(-1,0);
    lt43->SetLength(0.76);
    lt43->SetParameter(2);
    
    trackList.push_back(lt43);   

        //source 43
    LineTrack* lt44 = new LineTrack;//// S72
    lt44->SetWidth(0.2286);
    lt44->SetOrigin(-0.38,-1.15);// 
    lt44->SetDirection(-1,0);
    lt44->SetLength(1.16);
    lt44->SetParameter(2);
    
    trackList.push_back(lt44); 

        //source 44
    LineTrack* lt45 = new LineTrack;//// S75
    lt45->SetWidth(0.2286);
    lt45->SetOrigin(-1.54,-1.15);// 
    lt45->SetDirection(-1,0);
    lt45->SetLength(0.27);
    lt45->SetParameter(2);
    
    trackList.push_back(lt45); 

        //source 45
    LineTrack* lt46 = new LineTrack;//// Stop Area after S75
    lt46->SetWidth(0.2286);
    lt46->SetOrigin(-1.81,-1.15);// 
    lt46->SetDirection(-1,0);
    lt46->SetLength(0.2);
    lt46->SetParameter(2);
    
    trackList.push_back(lt46); 

            //source 46
    LineTrack* lt47 = new LineTrack;//// Starting point before S60
    lt47->SetWidth(0.2286);
    lt47->SetOrigin(2.51,-1.15);// 
    lt47->SetDirection(-1,0);
    lt47->SetLength(0.5);
    lt47->SetParameter(2);
    
    trackList.push_back(lt47);   

    //source 47
    ArcTrack* lt48 = new ArcTrack;//A108
    lt48->SetCenter(-2.032 -.01,0.3556+0.01); //(-2.032 -.01,0.3556+.2)
    lt48->SetRadius(0.6096-.01);
    lt48->SetAngleRange(3.0102,-1.172861);
    lt48->SetWidth(0.2286);
    lt48->SetDirection(false);
    lt48->SetRangeDirection(false);
    lt48->SetParameter(.2);

    trackList.push_back(lt48);
    
}
