#include "Srv_Upgrade.h"
#include "Srv_OsCommon.h"
#include "Bsp_Flash.h"
// #include "Storage.h"
#include "YModem.h"
#include "../common/util.h"
#include "CusQueue.h"
#include "../FCHW_Config.h"

#define UPGRADE_QUEUE_SIZE  (2 Kb)

#define PARA_TYPE           Para_Sys
#define PARA_NAME           "Upgrade_Info"

#if (CODE_TYPE == ON_BOOT)
#define FORCE_MODE_CODE     "Force_Mode"
#endif

/* internal function */

typedef struct
{
    SrvUpgrade_Send_Callback send;
    bool upgrade_on_bootup;
    uint8_t sw_ver[3];
    uint32_t firmware_size;
    uint8_t *firmware_buf;
    uint16_t pack_cnt;
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
#if (CODE_TYPE == ON_BOOT)
static bool SrvUpgrade_Load_Firmware(void);
static void SrvUpgrade_Check_ForceMode_Enable(void *arg);
#endif

/* external function */
static bool SrvUpgrade_Init(SrvUpgrade_Send_Callback tx_cb);
static void SrvUpgrade_DealRec(void *com_obj, uint8_t *p_data, uint16_t size);

/* external variable */
SrvUpgrade_TypeDef SrvUpgrade = {
    .init = SrvUpgrade_Init,
    .DealRec = SrvUpgrade_DealRec,
};

static bool SrvUpgrade_Init(SrvUpgrade_Send_Callback tx_cb)
{
    memset(&SrvUpgradeObj, 0, sizeof(SrvUpgradeObj_TypeDef));
    SrvUpgradeObj.init_state = false;
    SrvUpgradeObj.upgrade_on_bootup = false;
    SrvUpgradeObj.mode = Upgrade_Normal_Mode;
    SrvUpgradeObj.firmware_buf = SrvOsCommon.malloc(App_Section_Size);
    if (SrvUpgradeObj.firmware_buf == NULL)
        return false;

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
static void SrvUpgrade_Check_ForceMode_Enable(void *arg)
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
        
        if (SrvUpgradeObj.mode == Upgrade_Force_Mode)
            SrvUpgradeObj.send(arg, (uint8_t *)"\r\nswitch to force mode\r\n", strlen("\r\nswitch to force mode\r\n"));
        SrvUpgradeObj.queue_inuse = false;
    }
}
#endif

static void SrvUpgrade_Firmware_Rec_Start(void *arg, uint8_t p_data, uint16_t size)
{
    SrvUpgradeObj.pack_cnt = 0;
}

static void SrvUpgrade_Firmware_Rec_Pack(void *arg, uint8_t p_data, uint16_t size)
{
    /* update pack into ram */
    SrvUpgradeObj.pack_cnt ++;
}

static void SrvUpgrade_Firmware_Rec_Done(void *arg, uint8_t code)
{
    SrvUpgradeObj.pack_cnt = 0;

    if (code == YModem_Rx_Error)
    {

    }

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
    {
        SrvUpgradeObj.send(com_obj, p_data, size);
        return false;
    }
#endif

    /* Create YMdoem object */
    if (SrvUpgradeObj.YM_hdl == 0)
        SrvUpgradeObj.YM_hdl = YModem.Init(com_obj, SrvOsCommon.malloc, SrvOsCommon.free, \
                                           SrvUpgradeObj.send, \
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
    SrvUpgrade_Check_ForceMode_Enable(com_obj);
#endif

    SrvUpgrade_Firmware_Download(com_obj, p_data, size);

    SrvUpgradeObj.rec_cnt ++;
}

