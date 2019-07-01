

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

    //Tau=?
//    0.1813
//  ----------
//  z - 0.8187
//    SystemBlock filter(0.1813,0,- 0.8187,1);
//        SystemBlock filter(0,1,0,1);



    //passband
//    ans =
//       -0.0813         0    0.0813
//    ans =
//        0.8374   -1.8365    1.0000
//    SystemBlock filter(vector<double> {-0.0813   ,      0 ,   0.0813},
//                       vector<double> {0.8374  , -1.8365  ,  1.0000});


//    SystemBlock filterInput(filter);


    string folder="~/Escritorio";

    ofstream targets (folder+".targets.csv");
    ofstream responses (folder+".responses.csv");
    ofstream controls (folder+".controls.csv");



    SocketCanPort pm31("can1");
    CiA402SetupData sd31(2048,24,0.001, 0.144);
    CiA402Device m1 (31, &pm31, &sd31);
    m1.StartNode();
    m1.SwitchOn();
    PIDBlock c1(2,0.5,0,dts);

    SocketCanPort pm2("can1");
    CiA402Device m2 (32, &pm2);
    SocketCanPort pm3("can1");
    CiA402Device m3 (33, &pm3);



    double posan1, posan2, posan3;
    posan1=3.14;
    posan2=-posan1/2;
    posan3=-posan1/2;
    cout << "pos1 " << posan1  << ", pos2 " << posan2 << ", pos3 " << posan3 <<endl;

    IPlot p1,id;


    double ep1,ev1,cs1;
    double tp1,tv1,v1;
    double ep2,ev2,cs2;
    double ep3,ev3,cs3;

//    m1.Setup_Torque_Mode();
//    tv1=2;

    m1.Setup_Velocity_Mode();

    double input;

    double interval=4; //in seconds
    for (double t=0;t<interval; t+=dts)
    {

        tp1=3;

        ep1=tp1- m1.GetPosition();
        cs1= ep1 > c1;
//        cs1=cs1/10000;

//        cout << tp1 << endl;
        cs1=cs1-0.1*((rand() % 10 + 1)-5);
        m1.SetVelocity(cs1);
        v1 = (m1.GetVelocity() > filter);
        model.UpdateSystem(cs1 ,v1 );

        p1.pushBack(cs1 - v1);

//        model.GetZTransferFunction(num,den);
        model.PrintZTransferFunction(dts);



        //            cout << t << " , " << m1.GetPosition() << " , " << m2.GetPosition() <<  " , " << m3.GetPosition() <<endl;
        //            controls << t << " , " << cs1 << " , " << cs2 <<  " , " << cs3 <<endl;
        //            responses << t << " , " << m1.GetPosition() << " , " << m2.GetPosition() <<  " , " << m3.GetPosition() <<endl;

        //            cout << t << " , " << m1.GetVelocity() << " , " << m2.GetVelocity() <<  " , " << m3.GetVelocity() <<endl;
        //            responses << t << " , " << m1.GetVelocity() << " , " << m2.GetVelocity() <<  " , " << m3.GetVelocity() <<endl;

        tools.WaitSamplingTime();


    }

//    double interval=5; //in seconds
//    for (double t=0;t<interval; t+=dts)
//    {

//        ep1=tp1-m1.GetPosition();
//        cs1= ep1 > c1;
//        cs1=cs1+0.1*((rand() % 10 + 1)-5);
//        m1.SetVelocity(cs1);


//        model.UpdateSystem( cs1,m1.GetPosition() );
////        model.GetZTransferFunction(num,den);
//        model.PrintZTransferFunction(dts);

//        tools.WaitSamplingTime();

//    }

    vector<double> num(numOrder+1),den(denOrder+1);
    model.GetZTransferFunction(num,den);
    cout << num[1] << endl;
    SystemBlock idsys(num,den);

    for (double t=0; t<10; t+=dts)

    {

        id.pushBack( 1 > idsys );
        //Gz.PrintZTransferFunction(dts);
        //Gz.PrintParamsVector();

    }

    model.PrintZTransferFunction(dts);


    p1.Plot();
    id.Plot();


    m1.SetupPositionMode();
    m1.SetPosition(0);

    sleep(tp1);


targets.close();
controls.close();
responses.close();

return 0;

}
