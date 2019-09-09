

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

    //tau = 0.1
//    0.09516
//   ----------
//   z - 0.9048
    SystemBlock filter(0.09516,0,- 0.9048,1);
    int numOrder=0,denOrder=1;
    OnlineSystemIdentification model(numOrder,denOrder,filter);

    //Samplinfg time
    double dts=0.01;

    SamplingTime Ts;
    Ts.SetSamplingTime(dts);



//    model.SetFilter(filter);

    string folder="~/Escritorio/";

    ofstream targets (folder+"targets.csv");
    ofstream responses (folder+"responses.csv");
    ofstream controls (folder+"controls.csv");



    //m1 setup
    SocketCanPort pm31("can1");
    CiA402SetupData sd31(2048,24,0.001, 0.144);
    CiA402Device m1 (31, &pm31, &sd31);
    m1.StartNode();
    m1.SwitchOn();
    m1.Setup_Velocity_Mode(10);


    //m2
    SocketCanPort pm2("can1");
    CiA402Device m2 (32, &pm2);
    m2.StartNode();
    m2.SwitchOn();
    m2.Setup_Velocity_Mode(10);

    //m3
    SocketCanPort pm3("can1");
    CiA402Device m3 (33, &pm3);
    m3.StartNode();
    m3.SwitchOn();
    m3.Setup_Velocity_Mode(10);



    //controllers
//    PIDBlock c1(2,1,0,dts);
    FPDBlock c1(8.67,20.53,-0.83,dts);
    FPDBlock c2(8.67,20.53,-0.83,dts);
    FPDBlock c3(8.67,20.53,-0.83,dts);



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



    tp1=0;//*(1/interval)*t;



    double interval=6; //in seconds
    for (double t=0;t<interval; t+=dts)
    {


        ep1=tp1- m1.GetPosition();
        cs1= ep1 > c1;
//        cs1=cs1+0.01*((rand() % 10 + 1)-5);

//        cout << "target: " << tp1 << ", actual: " << m1.GetPosition() << endl;
//        cs1=cs1
        m1.SetVelocity(cs1);
        v1 = (m1.GetVelocity());
//        model.UpdateSystem(cs1 ,v1 );

//        p1.pushBack(v1 > filter);
        p1.pushBack(m1.GetPosition());


//        model.GetZTransferFunction(num,den);
//        model.PrintZTransferFunction(dts);


        Ts.WaitSamplingTime();


    }




    model.PrintZTransferFunction(dts);


//    id.Plot();
    p1.Plot();
//    p1.PlotAndSave("../pos.csv");

    m1.SetupPositionMode();
//    m1.SetPosition(0);

//    sleep(tp1);


targets.close();
controls.close();
responses.close();

return 0;

}

