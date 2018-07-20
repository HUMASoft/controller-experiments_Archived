

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


////    //fpd w=100 pm=80
//    vector<double> npd ={   33.3226,  143.5141,  347.1889,  451.1660 , 217.6672};
//    vector<double> dpd ={ 0.2520  ,  1.6080  ,  3.4552    ,3.0993   , 1.0000};


    //fpi w=25 pm=70 //kept from last experiments.
    vector<double> npi ={0.3354  ,  0.3724  , -1.8968 ,  -0.5654  ,  1.9306};
    vector<double> dpi ={0.2381  ,  0.3986  , -1.0645  , -0.5716  ,  1.0000};

    //fpd w=25 pm=70 //kept from last experiments.
//    vector<double> npd ={26.1251, -119.3468, -293.8182,  178.9746 , 361.7371};
//    vector<double> dpd ={-0.1440 , -0.1031 , 1.1445 , 2.1032 , 1.0000};

//    //w20p50isom scilab
//    vector<double> npd ={-5310.6413 ,  33607.557 , -52182.099 ,  23885.413};
//    vector<double> dpd ={0. ,  0.221738 , -1.1842456  , 1.};


//    //w15p100isom matlab
//    vector<double> npd ={  -0.0849 , -18.2048 , -24.0183,   32.3824  , 40.6971};
//    vector<double> dpd ={ -0.1826  , -0.4825  ,  0.4194   , 1.7166  ,  1.0000};

    //w10p80isom matlab
//    vector<double> npd ={    9.4033 ,  61.7897 , -96.8906 ,-120.3604 , 177.9012};
//    vector<double> dpd ={  -0.1189 ,  -0.3611   , 0.4540  ,  1.7011   , 1.0000};

    //w12p60isom fpi
    vector<double> npd ={        0.1903  ,  2.7145  , -3.0096  , -5.2954  ,  5.4174};
    vector<double> dpd ={        0.0321  ,  0.5141 ,  -0.5422 ,  -1.0022 ,   1.0000};


    SystemBlock pi1(npi,dpi);
    SystemBlock pi3(npi,dpi);
    SystemBlock pi2(npi,dpi);

//    PIDBlock pi1(1,100,0,dts);
//    PIDBlock pi2(1,100,0,dts);
//    PIDBlock pi3(1,100,0,dts);


    PIDBlock pd1(8.6120054,10.826259,0.2030172,dts);
    PIDBlock pd2(8.6120054,10.826259,0.2030172,dts);
    PIDBlock pd3(8.6120054,10.826259,0.2030172,dts);


//    SystemBlock pd1(npd,dpd,1);
//    SystemBlock pd2(npd,dpd,1);
//    SystemBlock pd3(npd,dpd,1);



    ofstream targets ("targets.csv");
    ofstream responses ("responses.csv");


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




    double ep1,ev1;
    double ep2,ev2;
    double ep3,ev3;

    double interval=2; //in seconds

//    double sats=80;
//    pd1.SetSaturation(-sats,sats);
//    pd2.SetSaturation(-sats*0.5,sats*0.5);
//    pd3.SetSaturation(-sats*0.5,sats*0.5);



    for (double t=0;t<interval; t+=dts)
    {
        if (onrobot)
        {
            ep1=posan1-m1.GetPosition();
            ev1= (ep1 > pd1)-m1.GetVelocity();
            m1.SetTorque((ev1 > pi1));

            ep2=posan2-m2.GetPosition();
            ev2= (ep2 > pd2)-m2.GetVelocity();
            m2.SetTorque((ev2 > pi2));

            ep3=posan3-m3.GetPosition();
            ev3= (ep3 > pd3)-m3.GetVelocity();
            m3.SetTorque(2.1*(ev3 > pi3));

            cout << t << " , " << m1.GetPosition() << " , " << m2.GetPosition() <<  " , " << m3.GetPosition() <<endl;
            responses << t << " , " << m1.GetPosition() << " , " << m2.GetPosition() <<  " , " << m3.GetPosition() <<endl;
//            cout << t << " , " << m1.GetVelocity() << " , " << m2.GetVelocity() <<  " , " << m3.GetVelocity() <<endl;
//            responses << t << " , " << m1.GetVelocity() << " , " << m2.GetVelocity() <<  " , " << m3.GetVelocity() <<endl;

        }
        usleep(dts*1000*1000);
        cout << t << " , " << posan1  << " , " << posan2 << " , " << posan3 << endl;
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


}
