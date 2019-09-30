

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
    SamplingTime Ts;
    Ts.SetSamplingTime(0.01);
    int numOrder=1,denOrder=1;
    OnlineSystemIdentification model(numOrder,denOrder);
//    vector<double> num(1),den(1);

    //Controllers
    double dts=0.01;


    //tau = 0.1
//    0.09516
//   ----------
//   z - 0.9048
    SystemBlock filter(0.09516,0,- 0.9048,1);


    //Tau=0.5
//    0.0198
//  ----------
//  z - 0.9802
//    SystemBlock filter(0.0198,0,- 0.9802,1);

    //Tau=??
//    0.1813
//  ----------
//  z - 0.8187
//    SystemBlock filter(0.1813,0,- 0.8187,1);

//    SystemBlock filterInput(filter);

    model.SetFilter(filter);

    string folder="~/Escritorio";

    ofstream targets (folder+".targets.csv");
    ofstream responses (folder+".responses.csv");
    ofstream controls (folder+".controls.csv");
    ofstream system (folder+".system.csv");



    SocketCanPort pm31("can1");
    CiA402SetupData sd31(2048,24,0.001, 0.144);
    CiA402Device m1 (31, &pm31, &sd31);
//    m1.StartNode();
//    m1.SwitchOn();
    PIDBlock c1(0.5,1,0,dts);

    SocketCanPort pm2("can1");
    CiA402Device m2 (32, &pm2);
//    m2.StartNode();
//    m2.SwitchOn();

    SocketCanPort pm3("can1");
    CiA402Device m3 (33, &pm3);
//    m3.StartNode();
//    m3.SwitchOn();

    IPlot p1,id;


    double ep1,ev1,cs1;
    double tp1,tv1,v1,tt1;
    double ep2,ev2,cs2;
    double ep3,ev3,cs3;

    m1.Setup_Torque_Mode();
    m2.Setup_Torque_Mode();
    m3.Setup_Torque_Mode();



    double pretorque = 0.0006;
    double torque = 0.00012;

    //torque to inclination id
    double interval=1; //in seconds
    for (double t=0;t<interval; t+=dts)
    {

        m1.SetTorque(pretorque);
        m2.SetTorque(pretorque);
        m3.SetTorque(pretorque+torque);

        model.UpdateSystem( tv1 ,v1 );

        p1.pushBack(v1);
        model.PrintZTransferFunction(dts);

        Ts.WaitSamplingTime();
    }




//    vector<double> num(numOrder+1),den(denOrder+1);
//    model.GetZTransferFunction(num,den);
//    SystemBlock idsys(num,den);

    for (double t=0; t<10; t+=dts)

    {

//        id.pushBack( 1 > idsys );
        //Gz.PrintZTransferFunction(dts);
        //Gz.PrintParamsVector();

    }





//    p1.Plot();
//    id.Plot();

    sleep(4);

targets.close();
controls.close();
responses.close();
system.close();

return 0;

}
