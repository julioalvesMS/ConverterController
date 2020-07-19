#ifndef SRC_CONVERTER_BOOST_H_
#define SRC_CONVERTER_BOOST_H_

#include <src/settings.h>
#include <src/Common/constants.h>
#include <src/Controller/controller.h>
#include <src/Converter/base_converter.h>
#include <src/SwitchedSystem/switched_system.h>

using namespace BaseConverter;
using namespace SwitchedSystem;

namespace ConverterBoost
{
    class Boost : public Converter
    {
    public:
        static System* GetSys(void);

        static System* GetDiscreteSys(void);

        static Cycle* GetLimitCycle(void);

        static void GetP(double P[SYSTEM_ORDER][SYSTEM_ORDER]);

        static void GetH(double h[SYSTEM_ORDER]);

        static double GetD(double P[SYSTEM_ORDER][SYSTEM_ORDER], double h[SYSTEM_ORDER]);

        static void GetClassicVoltageController(double num[2], double den[2]);

        static void GetClassicVoltageCurrentController(double vNum[2], double vDen[2], double iNum[2], double iDen[2]);

        static void GetStateFeedbackH2Controller(double K[2], double C[2], double* M);

        static void GetReferenceController(double num[2], double den[2]);

        static void GetCurrentCorrectionController(double num[2], double den[2]);

        static int SubSystem2SwitchState(int SubSystem);
    };

    void DefineSystem();

    void DefineDiscreteSystem();

    void DefineLimitCycleCost(void);

    void DefineLimitCycleH2(void);

    void DefineLimitCycleHinf(void);
}

#endif  /* SRC_CONVERTER_BOOST_H_ */
