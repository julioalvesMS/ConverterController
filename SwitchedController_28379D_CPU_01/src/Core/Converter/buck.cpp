#include <src/Core/Converter/buck.h>

using namespace Math;
using namespace SwitchedSystem;

namespace Buck
{
    static System system;

    System* GetSys()
    {
        DefineSystem();

        return &(system);
    }

    void DefineSystem()
    {
        SubSystem* subSys;

        //
        // =============== Subsystem 1 ===============
        //
        subSys = &(system.subSystems[0]);

        //
        // Subsystem 1 -- Matrix A
        //
        subSys->A.data[0][0] = -R/L;
        subSys->A.data[0][1] = -1/L;
        subSys->A.data[1][0] =  1/Co;
        subSys->A.data[1][1] = -1/(Ro*Co);
        //
        // Subsystem 1 -- Matrix B
        //
        subSys->B.data[0] = 1/L;
        subSys->B.data[1] = 0;
        //
        // Subsystem 1 -- Matrix Q
        //
        subSys->Q.data[0][0] = 0;
        subSys->Q.data[0][1] = 0;
        subSys->Q.data[1][0] = 0;
        subSys->Q.data[1][1] = 1/Ro;


        //
        // =============== Subsystem 2 ===============
        //
        subSys = &(system.subSystems[1]);

        //
        // Subsystem 2 -- Matrix A
        //
        subSys->A.data[0][0] = -R/L;
        subSys->A.data[0][1] = -1/L;
        subSys->A.data[1][0] =  1/Co;
        subSys->A.data[1][1] = -1/(Ro*Co);
        //
        // Subsystem 2 -- Matrix B
        //
        subSys->B.data[0] = 0;
        subSys->B.data[1] = 0;
        //
        // Subsystem 2 -- Matrix Q
        //
        subSys->Q.data[0][0] = 0;
        subSys->Q.data[0][1] = 0;
        subSys->Q.data[1][0] = 0;
        subSys->Q.data[1][1] = 1/Ro;
    }
}
