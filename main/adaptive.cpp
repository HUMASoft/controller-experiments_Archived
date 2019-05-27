

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
    OnlineSystemIdentification model(2,2);
    vector<double> num(2),den(2);

    //Controllers
    double dts=0.01;


    string folder="~/Escritorio";

    ofstream targets (folder+".targets.csv");
    ofstream responses (folder+".responses.csv");
    ofstream controls (folder+".controls.csv");



    SocketCanPort pm1("can1");
    CiA402SetupData setup1(4096,3.7,0.001,3);
    CiA402Device m1 (1, &pm1,setup1);
    m1.StartNode();
    m1.SwitchOn();

    SocketCanPort pm2("can1");
    CiA402Device m2 (2, &pm2);
    SocketCanPort pm3("can1");
    CiA402Device m3 (3, &pm3);



    double posan1, posan2, posan3;
    posan1=1.5;
    posan2=-posan1/2;
    posan3=-posan1/2;
    cout << "pos1 " << posan1  << ", pos2 " << posan2 << ", pos3 " << posan3 <<endl;



    double ep1,ev1,cs1;
    double ep2,ev2,cs2;
    double ep3,ev3,cs3;

    m1.SetupPositionMode(1, 1);
    ep1=posan1;

    double interval=6; //in seconds
    for (double t=0;t<interval; t+=dts)
    {

        m1.SetPosition(t/interval);


                    model.UpdateSystem( ep1,m1.GetPosition() );
                    model.GetZTransferFunction(num,den);
                    cout << "G=tf([ " << num[0] << ", " <<  num[1];
                    cout << "],[ " << den[0] << ", " <<  den[1] << "]," <<dts<< ")"<< endl;

        //            cout << t << " , " << m1.GetPosition() << " , " << m2.GetPosition() <<  " , " << m3.GetPosition() <<endl;
        //            controls << t << " , " << cs1 << " , " << cs2 <<  " , " << cs3 <<endl;
        //            responses << t << " , " << m1.GetPosition() << " , " << m2.GetPosition() <<  " , " << m3.GetPosition() <<endl;

        //            cout << t << " , " << m1.GetVelocity() << " , " << m2.GetVelocity() <<  " , " << m3.GetVelocity() <<endl;
        //            responses << t << " , " << m1.GetVelocity() << " , " << m2.GetVelocity() <<  " , " << m3.GetVelocity() <<endl;

        //usleep(uint(dts*1000*1000));
        tools.WaitSamplingTime();
        // cout << t << " , " << posan1  << " , " << posan2 << " , " << posan3 << endl;
        targets << t << " , " << posan1  << " , " << posan2 << " , " << posan3 << endl;

    }

    m1.SetPosition(0);

    sleep(4);

targets.close();
controls.close();
responses.close();

return 0;

}
