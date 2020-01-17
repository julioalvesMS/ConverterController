#ifndef SRC_CONVERTER_BASE_CONVERTER_H_
#define SRC_CONVERTER_BASE_CONVERTER_H_

#include <src/Common/constants.h>
#include <src/SwitchedSystem/switched_system.h>

using namespace SwitchedSystem;

namespace BaseConverter
{
    class Converter
    {
    public:
        static System* GetSys(void);

        static System* GetDiscreteSys(void);

        static void GetP(double P[SYSTEM_ORDER][SYSTEM_ORDER]);

        static void GetH(double h[SYSTEM_ORDER]);

        static double GetD(double P[SYSTEM_ORDER][SYSTEM_ORDER], double h[SYSTEM_ORDER]);

        static void GetClassicVoltageController(double num[2], double den[2]);

        static void GetClassicVoltageCurrnetController(double vNum[2], double vDen[2], double iNum[2], double iDen[2]);

        static void GetReferenceController(double num[2], double den[2]);

        static int SubSystem2SwitchState(int SubSystem);
    };

    enum ConverterID
    {
        ID_Buck = 0,
        ID_Boost = 1,
        ID_BuckBoost = 2,
        ID_BuckBoost3 = 3,
    };
}

#endif  /* SRC_CONVERTER_BASE_CONVERTER_H_ */
