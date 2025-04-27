#include "../FCHW_Config.h"
#include "Srv_OsCommon.h"
#include "util.h"
#include "kernel.h"
#include "HW_Def.h"
#include "Bsp_SDRAM.h"

typedef void (*Application_Func)(void);

typedef struct
{
    uint32_t available_size;
    uint32_t heap_remain;
    uint32_t block_num;

    uint32_t malloc_cnt;
    uint32_t malloc_failed_cnt;
    uint32_t free_cnt;
    uint32_t free_faile_cnt;

    bool sdram_state;
    uint32_t sdram_base_addr;
    uint32_t ext_mem_size;

    uint32_t t_w_cnt;           /* test write count */
    uint32_t t_w_failed_cnt;    /* test write failed count */
    
    BspSDRAMObj_TypeDef sdram_obj;
}SrvOsCommon_HeapMonitor_TypeDef;

/* internal vriable */
static SrvOsCommon_HeapMonitor_TypeDef OsHeap_Monitor = {0};

/* external function */
static void* SrvOsCommon_Malloc(uint32_t size);
static void SrvOsCommon_Free(void *ptr);
static void SrvOsCommon_Delay(uint32_t ms);
static void SrvOsCommon_DelayUntil(uint32_t *prev_time, uint32_t ms);
static bool SrvOsCommon_Init(void);
static void SrvOsCommon_JumpToAddr(uint32_t addr);

/* external vriable */
uint8_t __attribute__((section(".Os_Section"))) ucHeap[ configTOTAL_HEAP_SIZE ];

SrvOsCommon_TypeDef SrvOsCommon = {
    .init = SrvOsCommon_Init,
    .get_os_ms = osKernelSysTick,
    .delay_ms = SrvOsCommon_Delay,
    .precise_delay = SrvOsCommon_DelayUntil,
    .malloc = SrvOsCommon_Malloc,
    .free = SrvOsCommon_Free,
    .jump_to_addr = SrvOsCommon_JumpToAddr,
    .enter_critical = vPortEnterCritical,
    .exit_critical = vPortExitCritical,
    .get_heap_status = vPortGetHeapStats,
    .get_systimer_current_tick = Kernel_Get_SysTimer_TickUnit,
    .get_systimer_period = Kernel_Get_PeriodValue,
    .set_systimer_tick_value = Kernel_Set_SysTimer_TickUnit,
    .set_systimer_period = Kernel_Set_PeriodValue,
    .systimer_tick_to_us = Kernel_TickVal_To_Us,
    .systimer_disable = Kernel_DisableTimer_IRQ,
    .systimer_deinit = Kernel_BaseTick_DeInit,
    .systimer_enable = Kernel_EnableTimer_IRQ,
    .disable_all_irq = __disable_irq,
    .enable_all_irq = __enable_irq,
    .reboot = Kernel_reboot,
};

static bool SrvOsCommon_Init(void)
{
    uint32_t t_addr = 0;
    bool state = true;
    memset(&OsHeap_Monitor, 0, sizeof(SrvOsCommon_HeapMonitor_TypeDef));

    OsHeap_Monitor.sdram_state = false;
    OsHeap_Monitor.sdram_obj.hdl = malloc(SDRAM_HandleType_Size);
    if (OsHeap_Monitor.sdram_obj.hdl)
    {
        OsHeap_Monitor.sdram_obj.bank_num      = BspSDRAM_BankNum_4;
        OsHeap_Monitor.sdram_obj.bank_area     = BspSDRAM_Bank_1;
        OsHeap_Monitor.sdram_obj.bus_width     = BspSDRAM_BusWidth_16;
        OsHeap_Monitor.sdram_obj.column_bits   = BspSDRAM_Column_9Bits;
        OsHeap_Monitor.sdram_obj.row_bits      = BspSDRAM_Row_13Bits;

        /* init sdram if have */
        if (BspSDRAM_Init(&OsHeap_Monitor.sdram_obj))
        {
            /* sdram test */
            OsHeap_Monitor.sdram_state = true;
            OsHeap_Monitor.sdram_base_addr = 0xC0000000;
            
            t_addr = OsHeap_Monitor.sdram_base_addr;
            OsHeap_Monitor.t_w_failed_cnt = 0;
            for (OsHeap_Monitor.t_w_cnt = 0; OsHeap_Monitor.t_w_cnt < (configTOTAL_HEAP_SIZE / 2); OsHeap_Monitor.t_w_cnt ++)
            {
                *(volatile uint16_t *)t_addr = (uint16_t)OsHeap_Monitor.t_w_cnt;
                if (*(volatile uint16_t *)t_addr != (uint16_t)OsHeap_Monitor.t_w_cnt)
                {
                    state = false;
                    OsHeap_Monitor.sdram_state = false;
                    OsHeap_Monitor.t_w_failed_cnt ++;
                }

                /* reset value */
                *(volatile uint16_t *)t_addr = 0;
            }
        }
    }
    
    return state;
}

static void* SrvOsCommon_Malloc(uint32_t size)
{
    void *req_tmp = NULL;
    SrvOs_HeapStatus_TypeDef status; 
    uint32_t malloc_num = 0;
    memset(&status, 0, sizeof(SrvOs_HeapStatus_TypeDef));

    /* check heap status first */
    vPortGetHeapStats(&status);
    malloc_num = status.xNumberOfSuccessfulAllocations;
    
    if(status.xAvailableHeapSpaceInBytes)
    {
        req_tmp = pvPortMalloc(size);
        
        /* recheck heap status after os heap malloc */
        vPortGetHeapStats(&status);

        OsHeap_Monitor.available_size = status.xAvailableHeapSpaceInBytes;
        OsHeap_Monitor.block_num = status.xNumberOfFreeBlocks;
    
        if(!req_tmp && (status.xNumberOfSuccessfulAllocations - malloc_num != 1))
        {
            req_tmp = NULL;
            OsHeap_Monitor.malloc_failed_cnt ++;
        }
        else
        {
            memset(req_tmp, 0, size);
            OsHeap_Monitor.malloc_cnt ++;
        }
    }

    return req_tmp;
}

static void SrvOsCommon_Free(void *ptr)
{
    uint32_t free_cnt = 0;
    SrvOs_HeapStatus_TypeDef status;

    memset(&status, 0, sizeof(SrvOs_HeapStatus_TypeDef));

    if(ptr)
    {
        vPortGetHeapStats(&status);
        free_cnt = status.xNumberOfSuccessfulFrees;
        
        vPortFree(ptr);
        
        /* check heap status after os heap free */
        vPortGetHeapStats(&status);
        OsHeap_Monitor.available_size = status.xAvailableHeapSpaceInBytes;
        OsHeap_Monitor.block_num = status.xNumberOfFreeBlocks;

        if(status.xNumberOfSuccessfulFrees - free_cnt == 1)
        {
            OsHeap_Monitor.free_cnt++;
            ptr = NULL;
        }
        else
            OsHeap_Monitor.free_faile_cnt ++;
    }
}

static void SrvOsCommon_JumpToAddr(uint32_t addr)
{
    uint32_t jump_addr = 0;

    if ((addr < (uint32_t)&__boot_e) || ((addr & 0xFF000000) != (uint32_t)&__rom_s))
        return;

    jump_addr = *(volatile uint32_t *)(addr + 4);
    __set_MSP(*(volatile uint32_t *)addr);
    
    /* disable all irq before jump */
    SrvOsCommon.disable_all_irq();
    __set_CONTROL(0);

    ((Application_Func)jump_addr)();
}

static void SrvOsCommon_Delay(uint32_t ms)
{
    osDelay(ms);
}

static void SrvOsCommon_DelayUntil(uint32_t *prev_time, uint32_t ms)
{
    if (prev_time && ms)
        osDelayUntil(prev_time, ms);
}

// SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, reboot, Kernel_reboot, System ReBoot);

