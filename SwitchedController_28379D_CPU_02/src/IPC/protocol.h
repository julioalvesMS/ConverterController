#ifndef SRC_IPC_PROTOCOL_H_
#define SRC_IPC_PROTOCOL_H_

namespace Protocol
{
    enum CommunicationCommand
    {
        None = 0,

        EnableOperation,
        DisableOperation,

        IncreaseDacChannel,
        DecreaseDacChannel,

        RampIncreaseReference,
        RampDecreaseReference,

        StepIncreaseReference,
        StepDecreaseReference,

        ResetReference,

        EmergencyButtonProtection,
        ResetProtection,

        ConverterBuck,
        ConverterBoost,
        ConverterBuckBoost,
        ConverterBuckBoost3,

        ControllerClassic,
        ControllerContinuous1,
        ControllerContinuous2,
        ControllerDiscrete1,
        ControllerClassicVC,
        ControllerLimitCycleCost,
        ControllerLimitCycleH2,
        ControllerLimitCycleHinf,
        ControllerStateFeedbackH2,

        EquilibriumNone,
        EquilibriumReferenceController,
        EquilibriumPartialInformation,
        EquilibriumCurrentCorrection,

        EngageParallelLoad,
        DisengageParallelLoad,

        EnableModeHopping,
        DisableModeHopping,

        EnableLoadEstimation,
        DisableLoadEstimation,
    };
}

#endif /* SRC_IPC_PROTOCOL_H_ */
