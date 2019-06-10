


#include <iostream>
#include <fstream>

#include "FractionalDerivative.h"
#include "FractionalController1DOF.h"

#include "fcontrol.h"
#include "IPlot.h"
#include "OnlineSystemIdentification.h"



int main ()
{

    OnlineSystemIdentification id(1,2);

    //FractionalController1DOF fd(1,0.01);
    int N=100;
    double dts=0.01;
    vector<double> in(N,0),out(N,0);

    for (int i=0; i<N; i++)
    {
        in[i]=1;
        out[i]=dts;
//        cout << in[i] << ", ";
        id.UpdateSystem(in[i],out[i]);
        cout << id.PrintZTransferFunction(dts) << endl;
        //cout << id.PrintParamsVector() << endl;
    }

    for (int i=0; i<N; i++)
    {
        in[i]=1;
        out[i]=1;
//        cout << in[i] << ", ";
        id.UpdateSystem(in[i],out[i]);
        cout << id.PrintZTransferFunction(dts) << endl;
        //cout << id.PrintParamsVector() << endl;
    }

    for (int i=0; i<N; i++)
    {
//        out[i]=fd.OutputUpdate(in[i]);
//        cout << out[i] << ", ";

    }


}
