#include <src/IPC/ipc.h>


//
// Variables - Send
//
extern Protocol::CommunicationCommand IPC_CommandBuffer[];
extern int IPC_BufferIndex;

//
// Variables - Receive
//
extern int ADC_Vout, ADC_Vin, ADC_IL, ADC_Iout;
extern double Vin, Vout, IL, Iout;
extern double Vref;
extern double loadResistance;
extern int SwitchingFrequency;
extern bool ConverterEnabled;
extern bool ReferenceControlerEnabled;
extern bool OutputLoadStep;
extern int protection;
extern int DacChannel;
extern int activeConverter;
extern int controlStrategy;


//
// Shared Memory Variables - Send
//
#pragma DATA_SECTION("SHARERAMGS1");
volatile Protocol::CommunicationCommand shared_CommandBuffer[IPC_COMMAND_BUFFER_SIZE];

#pragma DATA_SECTION("SHARERAMGS1");
volatile int shared_BufferIndex;


//
// Shared Memory Variables - Receive
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
volatile double shared_Vref;

#pragma DATA_SECTION("SHARERAMGS0");
volatile int shared_SwitchingFrequency;

#pragma DATA_SECTION("SHARERAMGS0");
volatile bool shared_ConverterEnabled;

#pragma DATA_SECTION("SHARERAMGS0");
volatile bool shared_ReferenceControlerEnabled;

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


namespace IPC
{
    void Configure(void)
    {
        int i;
        for(i=0;i<IPC_COMMAND_BUFFER_SIZE; i++)
            IPC_CommandBuffer[i] = Protocol::None;
    }


    void SendData(void)
    {
        int i;
        for(i=0;i<IPC_COMMAND_BUFFER_SIZE; i++)
        {
            shared_CommandBuffer[i] = IPC_CommandBuffer[i];
        }

        shared_BufferIndex = IPC_BufferIndex;
    }


    void ReceiveData(void)
    {
        ADC_Vout = shared_ADC_Vout;
        ADC_Vin = shared_ADC_Vin;
        ADC_IL = shared_ADC_IL;
        ADC_Iout = shared_ADC_Iout;

        Vin = shared_Vin;
        Vout = shared_Vout;
        IL = shared_IL;
        Iout = shared_Iout;
        Vref = shared_Vref;
        loadResistance = shared_loadResistance;

        SwitchingFrequency = shared_SwitchingFrequency;
        ConverterEnabled = shared_ConverterEnabled;
        ReferenceControlerEnabled = shared_ReferenceControlerEnabled;
        OutputLoadStep = shared_OutputLoadStep;
        protection = shared_protection;
        activeConverter = shared_activeConverter;
        controlStrategy = shared_controlStrategy;
    }


    void CPUCommunication(void)
    {
        // If there is no pending flag, transfer data between the CPUs
        if(IPCRtoLFlagBusy(IPC_FLAG10) == 1)
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
            IPCRtoLFlagAcknowledge(IPC_FLAG10);
        }
    }
}
