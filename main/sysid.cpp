

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
    OnlineSystemIdentification model(1,1);
//    vector<double> num(1),den(1);

    //Controllers
    double dts=0.01;


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



    double posan1, posan2, posan3;
    posan1=3.14;
    posan2=-posan1/2;
    posan3=-posan1/2;
    cout << "pos1 " << posan1  << ", pos2 " << posan2 << ", pos3 " << posan3 <<endl;



    double ep1,ev1,cs1;
    double tp1,tv1,v1;
    double ep2,ev2,cs2;
    double ep3,ev3,cs3;

//    m1.Setup_Torque_Mode();
//    tv1=2;

    m1.Setup_Velocity_Mode();




    double interval=20; //in seconds
    for (double t=0;t<interval; t+=dts)
    {
        tv1=1*(rand() % 10 + 1)-5;

        cout << "tv1 " << tv1   <<endl;
        m1.SetVelocity(tv1);


        model.UpdateSystem( tv1,m1.GetPosition() );
        //        model.GetZTransferFunction(num,den);
        model.PrintZTransferFunction(dts);


        tools.WaitSamplingTime();

    }


    m1.SetupPositionMode(1,1);

    m1.SetPosition(0);

    sleep(4);

targets.close();
controls.close();
responses.close();

return 0;

}
