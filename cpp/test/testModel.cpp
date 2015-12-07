#include <iostream>
#include <fstream>

#include "config.h"

#include "ilqrsolver.h"
#include "costfunctionpneumaticarmelbow.h"
#include "pneumaticarmnonlinearmodel.h"
#include <time.h>
#include <sys/time.h>


using namespace std;
using namespace Eigen;

int main()
{
    struct timeval tbegin,tend;
    double texec=0.0;
    stateVec_t xinit,xDes,x;
    commandVec_t u;

    //xinit << 0.0,0.0,0.0,0.0;
    //xDes << 1.0,0.0,0.0,0.0;
    //xinit << 0.0,   0.0,    0.0,    4.0*1e5;
    //xDes << 1.0,    0.0,    2.0*1e5,    2.0*1e5;
    xinit << 0.2,0.0,0.0,4.0*1e5;
    xDes << 1.0,0.0,2.0*1e5,    2.0*1e5;
    u << 0.0,0.0;
    
    unsigned int T = 3000;
    double dt=5e-3;
   
    PneumaticarmNonlinearModel pneumaticarmModel(dt);




    ofstream fichier("/home/gkharish/softdev/DDP/cpp/src/resultsModel.csv",ios::out | ios::trunc);
    if(fichier)
    {
        fichier << "angular_position,angular_speed,angular_acceleration,u" << endl;
        x = xinit;
        fichier << x(0,0) << "," << x(1,0) << "," << x(2,0) << "," << x(3,0) << endl;
        for(int i=0;i<T;i++)
        {
            x = pneumaticarmModel.computeNextState(dt,x,u);
            fichier << x(0,0) << "," << x(1,0) << "," << x(2,0) << "," << x(3,0) << endl;
        }
        fichier.close();
    }
    else
        cerr << "erreur ouverte fichier" << endl;
    return 0;

}
