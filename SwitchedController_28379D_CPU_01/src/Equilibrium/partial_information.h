#ifndef SRC_EQUILIBRIUM_PARTIAL_INFORMATION_H_
#define SRC_EQUILIBRIUM_PARTIAL_INFORMATION_H_

extern double Vref;
extern double *Xe;

namespace PartialInformation
{
    void Configure(void);

    void LoadFilter(void);

    void UpdateReference(double IL);

    void ResetFilter(void);
}

#endif /* SRC_EQUILIBRIUM_PARTIAL_INFORMATION_H_ */
