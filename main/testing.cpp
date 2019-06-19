

#include <iostream>
#include <fstream>
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include "math.h"

#include "fcontrol.h"
#include "IPlot.h"

#include "OnlineSystemIdentification.h"


int main ()
{
    ToolsFControl tools;
    tools.SetSamplingTime(0.01);


    //Controllers
    double dts=0.01;


    SocketCanPort pm31("can1");
    CiA402SetupData sd31(2048,24,0.001, 0.144);
    CiA402Device m1 (31, &pm31, &sd31);
    m1.StartNode();
    m1.SwitchOn();

//    m1.Setup_Torque_Mode();
//    tv1=2;

    m1.Setup_Velocity_Mode(0,1);
    m1.SetVelocity(1);
    sleep(4);
    m1.SetVelocity(0);
//    m1.SetupPositionMode(1,1);
//    m1.SetPosition(0);
//    sleep(4);



return 0;

}
