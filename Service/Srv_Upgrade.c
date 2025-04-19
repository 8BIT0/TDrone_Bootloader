#include "Srv_Upgrade.h"
#include "Srv_OsCommon.h"
#include "Srv_ComTrans.h"
#include "Bsp_Flash.h"
// #include "Storage.h"
#include "YModem.h"
#include "../common/util.h"
#include "CusQueue.h"

#define UPGRADE_QUEUE_SIZE  (2 Kb)

#define PARA_TYPE           Para_Sys
#define PARA_NAME           "Upgrade_Info"
#define FORCE_MODE_CODE     "Force_Mode"

/* internal function */

typedef struct
{
    SrvUpgrade_Send_Callback send;
    bool upgrade_on_bootup;
    uint8_t sw_ver[3];
    uint16_t firmware_size;
    bool init_state;
    SrvUpgrade_Mode_TypeDef mode;
    uint32_t rec_cnt;

    bool queue_inuse;
    QueueObj_TypeDef p_queue;

    YModem_Handle YM_hdl;
} SrvUpgradeObj_TypeDef;

/* internal variable */
static SrvUpgradeObj_TypeDef SrvUpgradeObj;

/* internanl function */
static bool SrvUpgrade_Load_Firmware(void);
#if (CODE_TYPE == ON_BOOT)
static void SrvUpgrade_Check_ForceMode_Enable(void);
static void SrvUpgrade_JumpToApp(void);
#endif

/* external function */
static bool SrvUpgrade_Init(SrvUpgrade_Send_Callback tx_cb);
static void SrvUpgrade_DealRec(void *com_obj, uint8_t *p_data, uint16_t size);

/* external variable */
SrvUpgrade_TypeDef SrvUpgrade = {
    .init = SrvUpgrade_Init,
    .DealRec = SrvUpgrade_DealRec,
#if (CODE_TYPE == ON_BOOT)
   .JumpToApp = SrvUpgrade_JumpToApp,
#endif
};

static bool SrvUpgrade_Init(SrvUpgrade_Send_Callback tx_cb)
{
    memset(&SrvUpgradeObj, 0, sizeof(SrvUpgradeObj_TypeDef));
    SrvUpgradeObj.init_state = false;
    SrvUpgradeObj.upgrade_on_bootup = false;
    SrvUpgradeObj.mode = Upgrade_Normal_Mode;

    /* init queue */
    if (!Queue.create_auto(&SrvUpgradeObj.p_queue, "Upgrade_Queue", UPGRADE_QUEUE_SIZE))
        return false;
    
#if (CODE_TYPE == ON_BOOT)
    /* check storage system data section */
    /* check upgrade on boot up */

    /* init on chip flash */
    if (!BspFlash.init())
        return false;    
#endif

    SrvUpgradeObj.init_state = true;
    SrvUpgradeObj.send = tx_cb;

    return true;
}

/* copy firmware from back up area to on chip flash */
#if (CODE_TYPE == ON_BOOT)
static bool SrvUpgrade_Load_Firmware(void)
{
    if (!SrvUpgradeObj.init_state)
        return false;

    /* after firmware loaded */
    /* clear all data in storage section */

    return true;
}

/* if in boot mode check force code input */
static void SrvUpgrade_Check_ForceMode_Enable(void)
{
    uint16_t q_size = 0;
    uint8_t q_data = 0;

    if (!SrvUpgradeObj.init_state || (SrvUpgradeObj.mode >= Upgrade_Force_Mode))
        return;

    q_size = Queue.size(SrvUpgradeObj.p_queue);

    if (!SrvUpgradeObj.queue_inuse && (q_size >= strlen(FORCE_MODE_CODE)))
    {
        while (true)
        {
            SrvUpgradeObj.queue_inuse = true;

            if (!Queue.peek(&SrvUpgradeObj.p_queue, 0, &q_data, 1))
                break;

            if (q_data != FORCE_MODE_CODE[0])
            {
                Queue.pop(&SrvUpgradeObj.p_queue, &q_data, 1);
            }
            else
            {
                /* check queue remain size */
                q_size = Queue.size(SrvUpgradeObj.p_queue);
                if (q_size < strlen(FORCE_MODE_CODE))
                    break;
                    
                for (uint8_t i = 0; i < strlen(FORCE_MODE_CODE); i++)
                {
                    SrvUpgradeObj.mode = Upgrade_Normal_Mode;
                    Queue.pop(&SrvUpgradeObj.p_queue, &q_data, 1);

                    if (q_data != FORCE_MODE_CODE[i])
                        break;
                    
                   SrvUpgradeObj.mode = Upgrade_Force_Mode;
                }
            }
        }

        SrvUpgradeObj.queue_inuse = false;
    }
}

static void SrvUpgrade_JumpToAddr(void)
{
    uint32_t app_addr = *(volatile uint32_t *)(App_Address_Base + 4);
    void (*app_entry)(void) = (void (*)(void))app_addr;
    
    __set_MSP(*(volatile uint32_t *)App_Address_Base);
    SrvOsCommon.disable_all_irq();
    app_entry();
}
#endif

static void SrvUpgrade_Firmware_Rec_Start(void *arg, uint8_t p_data, uint16_t size)
{

}

static void SrvUpgrade_Firmware_Rec_Pack(void *arg, uint8_t p_data, uint16_t size)
{
    /* update pack into ram */
}

static void SrvUpgrade_Firmware_Rec_Done(void *arg, uint8_t code)
{
    if (code == YModem_Rx_Done)
    {
        SrvUpgradeObj.YM_hdl = 0;

        /* store firmware into external or on chip flash */
    }
}

static bool SrvUpgrade_Firmware_Download(void *com_obj, uint8_t *p_data, uint16_t size)
{
    if (!SrvUpgradeObj.init_state || (SrvUpgradeObj.send == NULL))
        return false;

#if (CODE_TYPE == ON_BOOT)
    if (SrvUpgradeObj.mode != Upgrade_Force_Mode)
        return false;
#endif

    /* Create YMdoem object */
    if (SrvUpgradeObj.YM_hdl == 0)
        SrvUpgradeObj.YM_hdl = YModem.Init(YModem_Rx_Pck, com_obj, \
                                           SrvOsCommon.malloc, SrvOsCommon.free, \
                                           (trans_callback)SrvCom.write, \
                                           NULL, SrvUpgrade_Firmware_Rec_Done, NULL);
    
    YModem.Rx(SrvUpgradeObj.YM_hdl, p_data, size);

    return true;
}

static void SrvUpgrade_DealRec(void *com_obj, uint8_t *p_data, uint16_t size)
{
    if ((!SrvUpgradeObj.init_state) || \
        (SrvUpgradeObj.mode > Upgrade_Force_Mode) || \
        (p_data == NULL))
        return;

    /* push data into queue */
    Queue.push(&SrvUpgradeObj.p_queue, p_data, size);

#if (CODE_TYPE == ON_BOOT)
    /* if in boot mode check force code input */
    SrvUpgrade_Check_ForceMode_Enable();
#endif

    SrvUpgrade_Firmware_Download(com_obj, p_data, size);

    SrvUpgradeObj.rec_cnt ++;
}

