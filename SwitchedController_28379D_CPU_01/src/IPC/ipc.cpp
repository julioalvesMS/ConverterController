#include <src/IPC/ipc.h>


//
// Variables - CPU2
//
extern Protocol::CommunicationCommand IPC_CommandBuffer[];
extern int IPC_BufferIndex;

//
// Variables - CPU1
//
extern double *Vin, *Vout, *IL, *Iout;
extern double *Vout_mean;
extern double Vref, VrefH;
extern double *loadResistance;
extern int SwitchingFrequency;
extern bool ConverterEnabled;
extern bool ModeHoppingEnabled;
extern bool LoadEstimationEnabled;
extern bool VoltageHolderEnabled;
extern int CorrectionMethod;
extern bool OutputLoadStep;
extern int protection;
extern int DacChannel;
extern int activeConverter;
extern int controlStrategy;
extern double stateDutyCycle[4];
extern double VoltageRipple;


//
// Shared Memory Variables - CPU2
//
#pragma DATA_SECTION("SHARERAMGS1");
volatile Protocol::CommunicationCommand shared_CommandBuffer[IPC_COMMAND_BUFFER_SIZE];

#pragma DATA_SECTION("SHARERAMGS1");
volatile int shared_BufferIndex;


//
// Shared Memory Variables - CPU1
//
#pragma DATA_SECTION("SHARERAMGS0");
volatile int shared_ADC_Vout;
#pragma DATA_SECTION("SHARERAMGS0");
volatile int shared_ADC_Vin;
#pragma DATA_SECTION("SHARERAMGS0");
volatile int shared_ADC_IL;
#pragma DATA_SECTION("SHARERAMGS0");
volatile int shared_ADC_Iout;

#pragma DATA_SECTION("SHARERAMGS0");
volatile double shared_Vin;
#pragma DATA_SECTION("SHARERAMGS0");
volatile double shared_Vout;
#pragma DATA_SECTION("SHARERAMGS0");
volatile double shared_IL;
#pragma DATA_SECTION("SHARERAMGS0");
volatile double shared_Iout;

#pragma DATA_SECTION("SHARERAMGS0");
volatile double shared_Vout_mean;

#pragma DATA_SECTION("SHARERAMGS0");
volatile double shared_Vref;
#pragma DATA_SECTION("SHARERAMGS0");
volatile double shared_VrefH;

#pragma DATA_SECTION("SHARERAMGS0");
volatile int shared_SwitchingFrequency;

#pragma DATA_SECTION("SHARERAMGS0");
volatile bool shared_ConverterEnabled;

#pragma DATA_SECTION("SHARERAMGS0");
volatile int shared_CorrectionMethod;

#pragma DATA_SECTION("SHARERAMGS0");
volatile bool shared_OutputLoadStep;

#pragma DATA_SECTION("SHARERAMGS0");
volatile int shared_protection;

#pragma DATA_SECTION("SHARERAMGS0");
volatile int shared_activeConverter;

#pragma DATA_SECTION("SHARERAMGS0");
volatile int shared_controlStrategy;

#pragma DATA_SECTION("SHARERAMGS0");
volatile double shared_loadResistance;

#pragma DATA_SECTION("SHARERAMGS0");
volatile double shared_stateDutyCycle[4];

#pragma DATA_SECTION("SHARERAMGS0");
volatile bool shared_ModeHoppingEnabled;

#pragma DATA_SECTION("SHARERAMGS0");
volatile bool shared_LoadEstimationEnabled;

#pragma DATA_SECTION("SHARERAMGS0");
volatile double shared_VoltageRipple;

#pragma DATA_SECTION("SHARERAMGS0");
volatile bool shared_VoltageHolderEnabled;


namespace IPC
{
    void Configure(void)
    {
        int i;
        for(i=0;i<IPC_COMMAND_BUFFER_SIZE; i++)
            IPC_CommandBuffer[i] = Protocol::None;
    }

    void ReceiveData(void)
    {
        int i;
        for(i=0;i<IPC_COMMAND_BUFFER_SIZE; i++)
        {
            IPC_CommandBuffer[i] = shared_CommandBuffer[i];
        }
        IPC_BufferIndex = shared_BufferIndex;
    }


    void SendData(void)
    {
        int i;

        shared_ADC_Vout = ADC_RESULT_VOUT;
        shared_ADC_Vin = ADC_RESULT_VIN;
        shared_ADC_IL = ADC_RESULT_IL;
        shared_ADC_Iout = ADC_RESULT_IOUT;

        shared_Vin = *Vin;
        shared_Vout = *Vout;
        shared_IL = *IL;
        shared_Iout = *Iout;
        shared_Vout_mean = *Vout_mean;
        shared_Vref = Vref;
        shared_VrefH = VrefH;
        shared_loadResistance = *loadResistance;

        shared_SwitchingFrequency = SwitchingFrequency;
        shared_ConverterEnabled = ConverterEnabled;
        shared_ModeHoppingEnabled = ModeHoppingEnabled;
        shared_LoadEstimationEnabled = LoadEstimationEnabled;
        shared_VoltageHolderEnabled = VoltageHolderEnabled;
        shared_CorrectionMethod = CorrectionMethod;
        shared_OutputLoadStep = OutputLoadStep;
        shared_protection = protection;
        shared_activeConverter = activeConverter;
        shared_controlStrategy = controlStrategy;
        shared_VoltageRipple = VoltageRipple;

        for(i=0;i<4;i++)
            shared_stateDutyCycle[i] = stateDutyCycle[i];
    }


    void CPUCommunication(void)
    {
        // If there is no pending flag, transfer data between the CPUs
        if(IPCLtoRFlagBusy(IPC_FLAG10) == 0)
        {
            //
            // Read data from CPU 1
            //
            ReceiveData();

            //
            // Write data to CPU 1
            //
            SendData();

            //
            // Set a flag to notify CPU02 that data is available
            //
            IPCLtoRFlagSet(IPC_FLAG10);
        }
    }
}
