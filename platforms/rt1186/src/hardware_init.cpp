#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_clock.h"

static void BOARD_InitADCClock(void)
{
    clock_root_config_t adc1ClkRoot;
    CLOCK_InitPfd(kCLOCK_PllSys2, kCLOCK_Pfd3, 19U);
    adc1ClkRoot.mux      = kCLOCK_ADC1_ClockRoot_MuxSysPll2Pfd3;
    adc1ClkRoot.div      = 6U;
    adc1ClkRoot.clockOff = false;
    CLOCK_SetRootClock(kCLOCK_Root_Adc1, &adc1ClkRoot);
}

void BOARD_InitHardware(void)
{
    BOARD_CommonSetting();
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitLEDsPins();
    BOARD_InitADCPins();
    BOARD_BootClockRUN();
    BOARD_InitADCClock();
    BOARD_InitDebugConsole();
    SystemCoreClockUpdate();
}
