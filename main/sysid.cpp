

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



    SocketCanPort pm31("can1");
    CiA402SetupData sd31(2048,24,0.001, 0.144);
    CiA402Device m1 (31, &pm31, &sd31);
    m1.StartNode();
    m1.SwitchOn();
    PIDBlock c1(0.5,1,0,dts);

    SocketCanPort pm2("can1");
    CiA402Device m2 (32, &pm2);
    SocketCanPort pm3("can1");
    CiA402Device m3 (33, &pm3);

    IPlot p1,id;


    double posan1, posan2, posan3;
    posan1=3.14;
    posan2=-posan1/2;
    posan3=-posan1/2;
    cout << "pos1 " << posan1  << ", pos2 " << posan2 << ", pos3 " << posan3 <<endl;



    double ep1,ev1,cs1;
    double tp1,tv1,v1,tt1;
    double ep2,ev2,cs2;
    double ep3,ev3,cs3;

//    m1.Setup_Torque_Mode();
//    tv1=2;

//    m1.SetupPositionMode();



    //velocity to velocity id
    m1.Setup_Velocity_Mode();
    double interval=10; //in seconds
    for (double t=0;t<interval; t+=dts)
    {
        tv1=0.5-0.5*((rand() % 10 + 1)-5);

//        cout << "tv1 " << tv1;
        m1.SetVelocity(tv1);

        v1 = m1.GetVelocity();// > filter;
//        model.UpdateSystem( tv1 > filterInput, v1 );

        model.UpdateSystem( tv1 ,v1 );

        p1.pushBack(v1);
        model.PrintZTransferFunction(dts);

        Ts.WaitSamplingTime();
    }


/*
    //torque to velocity id
    m1.Setup_Torque_Mode();
    double interval=5; //in seconds
    for (double t=0;t<interval; t+=dts)
    {
        tt1=+0.001*((rand() % 10 + 1)-5);

                cout << "tt1 " << tt1 << endl;
        m1.SetTorque(tt1);

        v1 = m1.GetVelocity() > filter;
        model.UpdateSystem( tt1, v1 );

        p1.pushBack(v1);
        //        tv1=tv1/10000;
        //        m1.SetTorque(tv1);
        //        model.UpdateSystem( tv1,m1.GetVelocity() );
        //        model.GetZTransferFunction(num,den);
        //        model.PrintZTransferFunction(dts);


        tools.WaitSamplingTime();
    }
*/


    m1.SetupPositionMode();
    m1.SetPosition(0);

    vector<double> num(numOrder+1),den(denOrder+1);
    model.GetZTransferFunction(num,den);
    SystemBlock idsys(num,den);

    for (double t=0; t<10; t+=dts)

    {

        id.pushBack( 1 > idsys );
        //Gz.PrintZTransferFunction(dts);
        //Gz.PrintParamsVector();

    }





    p1.Plot();
    id.Plot();

    sleep(4);

targets.close();
controls.close();
responses.close();

return 0;

}
