#ifndef SRC_CONVERTER_BASE_CONVERTER_H_
#define SRC_CONVERTER_BASE_CONVERTER_H_

#include <src/Common/constants.h>
#include <src/SwitchedSystem/switched_system.h>
#include <src/SwitchedSystem/cycle_sequence.h>

using namespace SwitchedSystem;
using namespace CycleSequence;

namespace BaseConverter
{
    class Converter
    {
    public:
        static System* GetSys(void);

        static System* GetDiscreteSys(void);

        static Cycle* GetLimitCycle(void);

        static void GetP(double P[SYSTEM_ORDER][SYSTEM_ORDER]);

        static void GetClassicVoltageController(double num[2], double den[2]);

        static void GetClassicVoltageCurrentController(double vNum[2], double vDen[2], double iNum[2], double iDen[2]);

        static void GetStateFeedbackH2Controller(double K[2], double C[2], double* M);

        static void GetReferenceController(double num[2], double den[2]);

        static void GetCurrentCorrectionController(double num[2], double den[2], double *designVoltage);

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
