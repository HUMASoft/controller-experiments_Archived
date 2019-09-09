

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



    TableInterpolation alpha("phi.m_exp.csv");



    cout << alpha.GetTableValue(30,12) << endl;




return 0;
}
