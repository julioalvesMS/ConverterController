#ifndef SRC_IPC_PROTOCOL_H_
#define SRC_IPC_PROTOCOL_H_

namespace Protocol
{
    enum CommunicationCommand
    {
        None = 0,

        EnableOperation = 1,
        DisableOperation = 2,

        IncreaseDacChannel = 3,
        DecreaseDacChannel = 4,

        RampIncreaseReference = 5,
        RampDecreaseReference = 6,

        StepIncreaseReference = 7,
        StepDecreaseReference = 8,

        EmergencyButtonProtection = 9,
        ResetProtection = 10,

        ConverterBuck = 11,
        ConverterBoost = 12,
        ConverterBuckBoost = 13,
        ConverterBuckBoost3 = 14,

        ControllerClassic = 15,
        ControllerContinuous1 = 16,
        ControllerContinuous2 = 17,
        ControllerDiscrete1 = 18,
        ControllerClassicVC = 19,
        ControllerLimitCycleCost = 20,
        ControllerLimitCycleH2 = 21,
        ControllerLimitCycleHinf = 22,

        EquilibriumNone = 23,
        EquilibriumReferenceController = 24,
        EquilibriumPartialInformation = 25,
        EquilibriumCurrentCorrection = 26,

        EngageParallelLoad = 27,
        DisengageParallelLoad = 28,
    };
}

#endif /* SRC_IPC_PROTOCOL_H_ */
