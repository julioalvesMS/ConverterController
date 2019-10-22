#ifndef SRC_CONVERTER_BUCK_BOOST_H_
#define SRC_CONVERTER_BUCK_BOOST_H_

#include <src/settings.h>
#include <src/Common/constants.h>
#include <src/Converter/base_converter.h>
#include <src/SwitchedSystem/switched_system.h>

using namespace BaseConverter;
using namespace SwitchedSystem;

namespace ConverterBuckBoost
{
    class BuckBoost : public Converter
    {
    public:
        static System* GetSys(void);

        static System* GetDiscreteSys(void);

        static void GetP(double P[SYSTEM_ORDER][SYSTEM_ORDER]);

        static void GetH(double h[SYSTEM_ORDER]);

        static double GetD(double P[SYSTEM_ORDER][SYSTEM_ORDER], double h[SYSTEM_ORDER]);

        static int SubSystem2SwitchState(int SubSystem);
    };

    void DefineSystem();

    void DefineDiscreteSystem();
}

#endif  /* SRC_CONVERTER_BUCK_BOOST_H_ */
