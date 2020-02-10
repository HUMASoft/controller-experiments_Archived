#include <iostream>
#include <fstream>
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include "math.h"

#include "SerialArduino.h"
#include "fcontrol.h"
#include "IPlot.h"


int main ()
{
    //--sensor--
    SerialArduino tilt;
    double incSensor,oriSensor;

    //Samplinfg time
    double dts=0.02;
    SamplingTime Ts(dts);


    PIDBlock incon(0.1,0.0,0,dts);
    PIDBlock orcon(0.01,0.0,0,dts);


    //m1 setup
    SocketCanPort pm31("can1");
    CiA402SetupData sd31(2048,24,0.001, 0.144);
    CiA402Device m1 (31, &pm31, &sd31);
    m1.Reset();
    m1.SwitchOn();
    //    m1.SetupPositionMode(5);
    m1.Setup_Velocity_Mode(5);


    //m2
    SocketCanPort pm2("can1");
    CiA402SetupData sd32(2048,24,0.001, 0.144);
    CiA402Device m2 (32, &pm2, &sd32);
    m2.Reset();
    m2.SwitchOn();
    //    m2.SetupPositionMode(5);
    m2.Setup_Velocity_Mode(5);

    //m3
    SocketCanPort pm3("can1");
    CiA402SetupData sd33(2048,24,0.001, 0.144);
    CiA402Device m3 (33, &pm3, &sd33);
    m3.Reset();
    //    m3.SwitchOn();
    //    m3.SetupPositionMode(5);
    //    m3.Setup_Velocity_Mode(5);





    double cs1;
    double cs2;
    double cs3;

    double ierror, oerror, ics = 0.0, ocs = 0.0;
    double radius=0.0075; //winch radius



    IPlot plot1,plot2;


//    double da2=2*M_PI/3, da3=4*M_PI/3; //angle shift for tendons 2 and 3


    //tilt initialization
    for (double t=0; t<6; t+=dts)
    {
        if (tilt.readSensor(incSensor,oriSensor)>=0) break;

    }


    double inc=20.0; //inclination tendon length
    double ori=290; //target orientation degrees


    double interval=30; //in seconds
    for (double t=0;t<interval; t+=dts)
    {
        if (tilt.readSensor(incSensor,oriSensor) <0)
        {
            cout << "Sensor error! " << endl;
            //Due to sensor error set motors zero velocity.
            m1.SetVelocity(0);
            m2.SetVelocity(0);
            m3.SetVelocity(0);

        }


        //negative feedback
        ierror = inc - incSensor;
        oerror = ori - oriSensor;

        ierror= ierror*M_PI/180; //degrees to rad
        oerror= oerror*M_PI/180; //degrees to rad

        //controller computes control signal
        ics = ierror > incon;
        ocs = oerror > orcon;

        if (!isnormal(ics)) ics = 0;
        if (!isnormal(ocs)||incSensor <5) ocs = 0;


        cs1=(ics-ocs)/radius;
        cs2=(ics+ocs)/radius;


        m1.SetVelocity(cs1);
        m2.SetVelocity(cs2);
        //            m3.SetVelocity(cs3);

        cout <<"t: "<< t << ", ierror " <<  ierror  << ", ics " << ics << ", incSensor " << incSensor <<endl;
        cout <<"t: "<< t << ", oerror " <<  oerror  << ", ocs " << ocs << ", oriSensor " << oriSensor <<endl;

        cout << "cs1 " << cs1 << ", cs2 " << cs2 /*<< ", cs3 " << cs3*/ <<endl;

        Ts.WaitSamplingTime();
    }

    m1.SetupPositionMode(5);
    m2.SetupPositionMode(5);
    m3.SetupPositionMode(5);


    //back to initial position
    interval = 5;
    for (double t=0;t<interval; t+=dts)
    {
        m1.SetPosition(0.01);
        m2.SetPosition(0.01);
        m3.SetPosition(0.01);
        cout<<"pos1: "<<m1.GetPosition()<<", "<<"pos2: "<<m2.GetPosition()<<", "<<"pos3: "<<m3.GetPosition()<<endl;
        Ts.WaitSamplingTime();
    }


    m1.Setup_Velocity_Mode(5);
    m2.Setup_Velocity_Mode(5);
    m3.Setup_Velocity_Mode(5);


    m1.SetVelocity(0);
    m2.SetVelocity(0);
    m3.SetVelocity(0);

    sleep(2);




    return 0;

}

