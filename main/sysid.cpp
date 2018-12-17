

#include <iostream>
#include <fstream>
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include "math.h"

#include "fcontrol.h"
#include "IPlot.h"



main ()
{
    //Controllers
    double dts=0.01;




    ofstream targets ("targets.csv");
    ofstream responses ("responses.csv");
    ofstream controls ("controls.csv");



    SocketCanPort pm1("can1");
    CiA402Device m1 (1, &pm1);
    SocketCanPort pm2("can1");
    CiA402Device m2 (2, &pm2);
    SocketCanPort pm3("can1");
    CiA402Device m3 (3, &pm3);



    m1.Setup_Torque_Mode();
    m2.Setup_Torque_Mode();
    m3.Setup_Torque_Mode();



    double ep1,ev1,cs1;
    double ep2,ev2,cs2;
    double ep3,ev3,cs3;

    double interval=3; //in seconds
    double tor1=300;

    //    double sats=40;
    //    pd1.SetSaturation(-sats,sats);
    //    pd2.SetSaturation(-sats,sats);
    //    pd3.SetSaturation(-sats,sats);

        m1.SetTorque(tor1);

//        m2.SetTorque(tor1);

//    m3.SetTorque(tor1);


    for (double t=0;t<interval; t+=dts)
    {




        //  cout << t << " , " << m1.GetPosition() << " , " << m2.GetPosition() <<  " , " << m3.GetPosition() <<endl;
//        controls << t << " , " << cs1 << " , " << cs2 <<  " , " << cs3 <<endl;
//        responses << t << " , " << m1.GetPosition() << " , " << m2.GetPosition() <<  " , " << m3.GetPosition() <<endl;

        //            cout << t << " , " << m1.GetVelocity() << " , " << m2.GetVelocity() <<  " , " << m3.GetVelocity() <<endl;
                    responses << t << " , " << m1.GetVelocity() << " , " << m2.GetVelocity() <<  " , " << m3.GetVelocity() <<endl;

        usleep(dts*1000*1000);
        // cout << t << " , " << posan1  << " , " << posan2 << " , " << posan3 << endl;

    }



    m1.SetTorque(-tor1);

    m1.SetTorque(0);

    m2.SetTorque(0);
    m3.SetTorque(0);

//    sleep(1);

    targets.close();
    controls.close();
    responses.close();

}
