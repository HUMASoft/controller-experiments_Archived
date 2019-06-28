

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
    //Controllers
    double dts=0.01;
    double vel;


    ToolsFControl tools;
    tools.SetSamplingTime(dts);

    //tau = 0.1
//    0.09516
//   ----------
//   z - 0.9048
//    SystemBlock filter(0.09516,0,- 0.9048,1);


    //Tau=0.5
//    0.0198
//  ----------
//  z - 0.9802
//    SystemBlock filter(0.0198,0,- 0.9802,1);

//    0.1813
//  ----------
//  z - 0.8187
    SystemBlock filter(0.1813,0,- 0.8187,1);

    IPlot plot;




    SocketCanPort pm31("can1");
    CiA402SetupData sd31(2048,24,0.001, 0.144);
    CiA402Device m1 (31, &pm31, &sd31);
    m1.StartNode();
    m1.SwitchOn();

//    m1.Setup_Torque_Mode();
//    tv1=2;

    m1.Setup_Velocity_Mode();
    m1.SetVelocity(1);

    for (double t=0; t<2;t+=dts)
    {
        vel =  m1.GetVelocity() > filter;
        plot.pushBack( vel );
        cout << "vel =  " << vel << endl;
        tools.WaitSamplingTime();
    }

    plot.Plot();


//    sleep(4);
    m1.SetVelocity(0);
    m1.SetupPositionMode(1,1);
    m1.SetPosition(0);
    sleep(4);



return 0;

}
