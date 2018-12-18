


#include <iostream>
#include <fstream>
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include "math.h"
#include "FractionalDerivative.h"

#include "fcontrol.h"
#include "IPlot.h"



int main ()
{

    FractionalDerivative fd(1.5,0.01);
    int N=100;
    double dts=0.01;
    vector<double> in(N,0),out(N,0);

    for (int i=0; i<N; i++)
    {
        in[i]=dts;
//        cout << in[i] << ", ";

    }

    for (int i=0; i<N; i++)
    {
        out[i]=fd.OutputUpdate(in[i]);
        cout << out[i] << ", ";

    }


}
