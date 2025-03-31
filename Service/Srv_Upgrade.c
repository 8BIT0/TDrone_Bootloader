#include "Srv_Upgrade.h"
#include "Srv_OsCommon.h"
#include "Srv_ComTrans.h"
#include "Bsp_Flash.h"
#include "Storage.h"
#include "YModem.h"
#include "../common/util.h"
#include "CusQueue.h"

#define UPGRADE_QUEUE_SIZE  (2 Kb)

#define ON_BOOT             1
#define ON_APP              2

#define CODE_TYPE           ON_BOOT
#define PARA_TYPE           Para_Sys
#define PARA_NAME           "Upgrade_Info"
#define FORCE_MODE_CODE     "Force_Mode\r\n"

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
} SrvUpgradeObj_TypeDef;

/* internal variable */
static SrvUpgradeObj_TypeDef SrvUpgradeObj;

/* internanl function */
static bool SrvUpgrade_Init(SrvUpgrade_Send_Callback tx_cb);

/* external function */

/* external variable */
SrvUpgrade_TypeDef SrvUpgrade = {
    .init = SrvUpgrade_Init,
};

static bool SrvUpgrade_Init(SrvUpgrade_Send_Callback tx_cb)
{
    memset(&SrvUpgradeObj, 0, sizeof(SrvUpgradeObj_TypeDef));
    SrvUpgradeObj.init_state = false;
    SrvUpgradeObj.upgrade_on_bootup = false;
    SrvUpgradeObj.mode = Upgrade_Normal_Mode;

#if (CODE_TYPE == ON_BOOT)
    /* init on chip flash */
    if (!BspFlash.init())
        return false;
#endif

    /* init queue */
    if (!Queue.create_auto(&SrvUpgradeObj.p_queue, "Upgrade_Queue", UPGRADE_QUEUE_SIZE))
        return false;

    /* check storage system data section */
    /* check upgrade on boot up */

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
#endif

#if (CODE_TYPE == ON_BOOT)
/* if in boot mode check force code input */
static void SrvUpgrade_Check_ForceMode_Enable(uint8_t *p_data, uint16_t size)
{
    if ((p_data == NULL) || (size == 0) || (SrvUpgradeObj.mode >= Upgrade_Force_Mode))
        return;

    if (!SrvUpgradeObj.queue_inuse)
    {

    }
}

#endif

static void SrvUpgrade_DealRec(uint8_t *p_data, uint16_t size)
{
    if ((!SrvUpgradeObj.init_state) || \
        (SrvUpgradeObj.mode > Upgrade_Force_Mode) || \
        (p_data == NULL)|| (size == 0))
        return;

#if (CODE_TYPE == ON_BOOT)
    /* if in boot mode check force code input */
    SrvUpgrade_Check_ForceMode_Enable(p_data, size);
#endif

    SrvUpgradeObj.rec_cnt ++;
}

