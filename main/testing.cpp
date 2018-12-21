


#include <iostream>
#include <fstream>

#include "FractionalDerivative.h"
#include "FractionalController1DOF.h"

#include "fcontrol.h"
#include "IPlot.h"
#include "OnlineSystemIdentification.h"



int main ()
{

    OnlineSystemIdentification id;
    FractionalController1DOF fd(1,0.01);
    int N=100;
    double dts=0.01;
    vector<double> in(N,0),out(N,0);

    for (int i=0; i<N; i++)
    {
        in[i]=dts;
//        cout << in[i] << ", ";
        id.UpdateSystem(in[i],out[i]);

    }

    for (int i=0; i<N; i++)
    {
//        out[i]=fd.OutputUpdate(in[i]);
        cout << out[i] << ", ";

    }


}
