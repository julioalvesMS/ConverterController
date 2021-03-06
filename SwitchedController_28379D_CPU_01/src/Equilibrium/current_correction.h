#ifndef SRC_EQUILIBRIUM_CURRENT_CORRECTION_H_
#define SRC_EQUILIBRIUM_CURRENT_CORRECTION_H_

#include <src/Equilibrium/equilibrium.h>

extern double Vref;
extern double *Xe;

namespace CurrentCorrection
{
    void Configure(void);

    void LoadController(void);

    void UpdateReference(double Vout, double u, double Rom, bool enable);

    double ModulateCorrection(double Ie, double u, double Rom);

    void ResetController(void);
}

#endif /* SRC_EQUILIBRIUM_CURRENT_CORRECTION_H_ */
