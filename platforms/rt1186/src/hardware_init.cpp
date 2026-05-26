#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"

void BOARD_InitHardware(void)
{
    BOARD_CommonSetting();
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitLEDsPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    /* Update the core clock */
    SystemCoreClockUpdate();
}
