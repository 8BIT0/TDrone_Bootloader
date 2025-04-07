#include "Task_Manager.h"
#include "debug_util.h"
#include "HW_Def.h"
#include "Dev_Led.h"
#include "Srv_OsCommon.h"
#include "shell_port.h"
#include "cmsis_os.h"
#include "Srv_ComTrans.h"
#include "Srv_Upgrade.h"

#define WINDOW_SIZE 100
#define RECEIVE_TIME_OUT 1000

osThreadId TaskManager_Handle = NULL;

#define SYS_TAG "[ SYS INFO ] "
#define SYS_INFO(fmt, ...) Debug_Print(&DebugPort, SYS_TAG, fmt, ##__VA_ARGS__)

/* internal vairable */
static uint32_t time_out_tick = WINDOW_SIZE;
static SrvComObj_TypeDef SrvComObj;

void Task_Manager_Init(void)
{
    DevLED.init(Led1);
    SrvOsCommon.init();

    osThreadDef(MainTask, Task_Main_Logic, osPriorityLow, 0, (1 Kb));
    TaskManager_Handle = osThreadCreate(osThread(MainTask), NULL);

    osKernelStart();
}

void Task_Main_Logic(void const *arg)
{
    uint32_t sys_time = 0;

    DebugPort.free = SrvOsCommon.free;
    DebugPort.malloc = SrvOsCommon.malloc;
    Debug_Port_Init(&DebugPort);

    SYS_INFO("%s\r\n", Select_Hardware);
    SYS_INFO("Hardware Version %d.%d.%d", HWVer[0], HWVer[1], HWVer[2]);
    SYS_INFO("Bootloader", "Start");
    
    SrvCom.init(&SrvComObj);
    SrvUpgrade.init(SrvCom.write);
    
    while(1)
    {
        sys_time = SrvOsCommon.get_os_ms();
        if (time_out_tick == 0)
            time_out_tick = sys_time + RECEIVE_TIME_OUT; 
        
        /* bootloader logic loop */

        /* run system statistic in this task */
        osDelay(10);
    }
}

static void Bootloader_Check(uint32_t sys_time)
{
    uint16_t rec_size = 0;

    /* nothing receive when 100ms when power on */
    if (sys_time >= time_out_tick)
    {
        SYS_INFO("Wait Firmware", "Time Out");
        SYS_INFO("Bootloader exit", "Jump to APP");

        SrvOsCommon.delay_ms(10);

        /* jump to app */
        // SrvOsCommon.jump_to_addr();
    }
    else
    {
        /* check com port received size */
        if (SrvCom.available)
            rec_size = SrvCom.available(SrvComObj);

        if (rec_size)
        {
            /* update time out tick */
            time_out_tick = sys_time + RECEIVE_TIME_OUT;
        }
    }
}

