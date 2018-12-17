

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
    bool onrobot=true;
    //Controllers
    double dts=0.01;



    //fpi w=25 pm=70 //kept from last experiments.
    vector<double> npi ={-0.2905,    1.1836 ,  -1.5196 ,   0.6267};
    vector<double> dpi ={-0.9411,    2.8818 ,  -2.9407   , 1.0000};


//    SystemBlock pi1(npi,dpi);
//    SystemBlock pi3(npi,dpi);
//    SystemBlock pi2(npi,dpi);
//    pi2.SetSaturation(-700,700);
    //string method("w12p60pid");
    PIDBlock pi1(8.6120054,10.826259,0.2030172,dts);
    PIDBlock pi2(0.165,21.15,0,dts);
    PIDBlock pi3(8.6120054,10.826259,0.2030172,dts);

    string mass("/home/humasoft/Escritorio/");



    //w12p60isom fpi

//    string method("w12p60isom");
//    vector<double> npd ={0.1903  ,  2.7145  , -3.0096  , -5.2954  ,  5.4174};
//    vector<double> dpd ={0.0321  ,  0.5141 ,  -0.5422 ,  -1.0022 ,   1.0000};
//    SystemBlock pd1(npd,dpd,1);
//    SystemBlock pd2(npd,dpd,1);
//    SystemBlock pd3(npd,dpd,1);



//    w12p60monje fpi

    string method("w12p60monje");
    vector<double> npd ={-41.8156,  226.0708, -488.6249,  527.7509, -284.8347,   61.4535};
    vector<double> dpd ={0.6126,   -2.1962,    1.9705,    1.1962 ,  -2.5831 ,   1.0000};
//    SystemBlock pd1(npd,dpd,1);
//    SystemBlock pd2(npd,dpd,1);
//    SystemBlock pd3(npd,dpd,1);
   // pd1.SetSaturation(-200,200)



//    //w12p60pid

   //string method("w12p60pid");
    PIDBlock pd1(8.6120054,10.826259,0.2030172,dts);
    PIDBlock pd2(21.65,0,1.9,dts);
    PIDBlock pd3(8.6120054,10.826259,0.2030172,dts);



    ofstream targets (mass+method+".targets.csv");
    ofstream responses (mass+method+".responses.csv");
    ofstream controls (mass+method+".controls.csv");



        SocketCanPort pm1("can0");
        CiA402Device m1 (1, &pm1);
        SocketCanPort pm2("can0");
        CiA402Device m2 (2, &pm2);
        SocketCanPort pm3("can0");
        CiA402Device m3 (3, &pm3);

        if (onrobot)
        {
//  Remember to switch on before to keep this commented.

//        m1.Reset();
//        m2.Reset();
//        m3.Reset();

//        m1.SwitchOn();
//        sleep(1);
//        m2.SwitchOn();
//        sleep(1);
//        m3.SwitchOn();
//        sleep(1);

        m1.Setup_Torque_Mode();
        m2.Setup_Torque_Mode();
        m3.Setup_Torque_Mode();
    }

//    TableKinematics a;
//    vector<double> lengths(3);
//    long orient=1;
//    long incli=1;

//    a.GetIK(incli,orient,lengths);
//    cout << "l1 " << lengths[0]  << ", l2 " << lengths[1] << ", l3 " << lengths[2]<<endl;
//    double posan1, posan2, posan3;
//    posan1=(0.1-lengths[0])*180/(0.01*M_PI);
//    posan2=(0.1-lengths[1])*180/(0.01*M_PI);
//    posan3=(0.1-lengths[2])*180/(0.01*M_PI);
double posan1, posan2, posan3;
    posan1=80;
    posan2=-posan1/2;
    posan3=-posan1/2;
    cout << "pos1 " << posan1  << ", pos2 " << posan2 << ", pos3 " << posan3 <<endl;



    double ep1,ev1,cs1;
    double ep2,ev2,cs2;
    double ep3,ev3,cs3;

    double interval=2; //in seconds

//    double sats=40;
//    pd1.SetSaturation(-sats,sats);
//    pd2.SetSaturation(-sats,sats);
//    pd3.SetSaturation(-sats,sats);



    for (double t=0;t<interval; t+=dts)
    {
        if (onrobot)
        {
            ep1=posan1-m1.GetPosition();
            cs1=ep1 > pd1;
            ev1= cs1-m1.GetVelocity();
            m1.SetTorque((ev1 > pi1));

            ep2=posan2-m2.GetPosition();
            cs2=ep2 > pd2;
            ev2= cs2-m2.GetVelocity();
            m2.SetTorque(0.8*(ev2 > pi2));

            ep3=posan3-m3.GetPosition();
            cs3=ep3 > pd3;
            ev3= cs3-m3.GetVelocity();
            m3.SetTorque(2.1*(ev3 > pi3));

          //  cout << t << " , " << m1.GetPosition() << " , " << m2.GetPosition() <<  " , " << m3.GetPosition() <<endl;
            controls << t << " , " << cs1 << " , " << cs2 <<  " , " << cs3 <<endl;
            responses << t << " , " << m1.GetPosition() << " , " << m2.GetPosition() <<  " , " << m3.GetPosition() <<endl;

//            cout << t << " , " << m1.GetVelocity() << " , " << m2.GetVelocity() <<  " , " << m3.GetVelocity() <<endl;
//            responses << t << " , " << m1.GetVelocity() << " , " << m2.GetVelocity() <<  " , " << m3.GetVelocity() <<endl;

        }
        usleep(dts*1000*1000);
       // cout << t << " , " << posan1  << " , " << posan2 << " , " << posan3 << endl;
        targets << t << " , " << posan1  << " , " << posan2 << " , " << posan3 << endl;

    }


    if (onrobot)
    {
        posan1=000;
        posan2=-posan1/2;
        posan3=-posan1/2;
        for (double t=0;t<interval; t+=dts)
        {

                ep1=posan1-m1.GetPosition();
                ev1= (ep1 > pd1)-m1.GetVelocity();
                m1.SetTorque(ev1 > pi1);

                ep2=posan2-m2.GetPosition();
                ev2= (ep2 > pd2)-m2.GetVelocity();
                m2.SetTorque(ev2 > pi2);

                ep3=posan3-m3.GetPosition();
                ev3= (ep3 > pd3)-m3.GetVelocity();
                m3.SetTorque(2.1*(ev3 > pi3));



            usleep(dts*1000*1000);

        }

        m1.SetTorque(0);
        m2.SetTorque(0);
        m3.SetTorque(0);

        sleep(1);
     }
targets.close();
controls.close();
responses.close();

}
