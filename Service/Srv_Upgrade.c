#include "Srv_Upgrade.h"
#include "Srv_OsCommon.h"
#include "Bsp_Flash.h"
#include "../System/storage/Storage.h"
#include "YModem.h"
#include "../common/util.h"
#include "CusQueue.h"
#include "../FCHW_Config.h"
#include "HW_Def.h"

#define UPGRADE_QUEUE_SIZE      (2 Kb)

#define PARA_TYPE               Para_Sys
#define PARA_NAME               "Upgrade_Info"

#if (CODE_TYPE == ON_BOOT)
#define FORCE_MODE_CODE         "force_mode"    /* force to receive firmware mode */
#endif

#define FIRMWARE_DUMP_SIZE      (1 Kb)

typedef struct
{
    SrvUpgrade_Send_Callback send;
    bool upgrade_on_bootup;
    uint8_t *firmware_name;
    uint32_t firmware_size;
    uint32_t firmware_rec_size;
    uint8_t *firmware_buf;
    uint16_t pack_ok_cnt;
    uint16_t pack_err_cnt;
    bool init_state;
    SrvUpgrade_Mode_TypeDef mode;
    uint32_t rec_cnt;

    bool queue_inuse;
    QueueObj_TypeDef p_queue;
    SrvUpgrade_BaseInfo_TypeDef FirmwareInfo;
    Storage_ItemSearchOut_TypeDef SearchOut;

    YModem_Handle YM_hdl;
} SrvUpgradeObj_TypeDef;

/* internal variable */
static SrvUpgradeObj_TypeDef SrvUpgradeObj;

/* internanl function */
#if (CODE_TYPE == ON_BOOT)
static bool SrvUpgrade_Upgrade_Firmware(uint32_t firmware_size);
static void SrvUpgrade_Check_ForceMode_Enable(void *arg);
#endif

/* external function */
static bool SrvUpgrade_Init(SrvUpgrade_Send_Callback tx_cb);
static void SrvUpgrade_Set_TransCallback(SrvUpgrade_Send_Callback tx_cb);
static void SrvUpgrade_DealRec(void *com_obj, uint8_t *p_data, uint16_t size);
static bool SrvUpgrade_DumpFirmware(SrvUpgrade_FirmwareDumpType_List type, void *port, SrvUpgrade_Send_Callback tx_cb);

/* external variable */
SrvUpgrade_TypeDef SrvUpgrade = {
    .init = SrvUpgrade_Init,
    .DealRec = SrvUpgrade_DealRec,
    .DumpFirmware = SrvUpgrade_DumpFirmware,
    .set_trans_callback = SrvUpgrade_Set_TransCallback,
};

static bool SrvUpgrade_Init(SrvUpgrade_Send_Callback tx_cb)
{
    uint16_t info_size = 0;
    Storage_ItemSearchOut_TypeDef *p_search = NULL;

    memset(&SrvUpgradeObj.FirmwareInfo, 0, sizeof(SrvUpgrade_BaseInfo_TypeDef));
    memset(&SrvUpgradeObj.SearchOut, 0, sizeof(Storage_ItemSearchOut_TypeDef));
    memset(&SrvUpgradeObj, 0, sizeof(SrvUpgradeObj_TypeDef));
    
    p_search = &SrvUpgradeObj.SearchOut;
    SrvUpgradeObj.init_state = false;
    SrvUpgradeObj.upgrade_on_bootup = false;
    SrvUpgradeObj.mode = Upgrade_Normal_Mode;
    SrvUpgradeObj.firmware_buf = SrvOsCommon.malloc(App_Firmware_Size);
    if (SrvUpgradeObj.firmware_buf == NULL)
        return false;

    /* init queue */
    if (!Queue.create_auto(&SrvUpgradeObj.p_queue, "Upgrade_Queue", UPGRADE_QUEUE_SIZE))
        return false;
    
#if (CODE_TYPE == ON_BOOT)
    /* init stm32h743 internal flash */
    if (!BspFlash.init())
        return false;

    /* check storage system data section */
    *p_search = Storage.search(PARA_TYPE, PARA_NAME);
    if (p_search->item_addr == 0)
    {
        /* first time create section */
        Storage.create(PARA_TYPE, PARA_NAME, (uint8_t *)&SrvUpgradeObj.FirmwareInfo, sizeof(SrvUpgrade_BaseInfo_TypeDef));
    }
    else
    {
        /* search data */
        if ((Storage.get(PARA_TYPE, p_search->item, (uint8_t *)&SrvUpgradeObj.FirmwareInfo, &info_size) != Storage_Error_None) || \
            (info_size != sizeof(SrvUpgrade_BaseInfo_TypeDef)))
        {
            SrvUpgradeObj.FirmwareInfo.compelet = false;
            SrvUpgradeObj.FirmwareInfo.update = false;
        }

        /* check upgrade on boot up */
        if ((SrvUpgradeObj.FirmwareInfo.compelet & SrvUpgradeObj.FirmwareInfo.update) && \
             SrvUpgrade_Upgrade_Firmware(SrvUpgradeObj.FirmwareInfo.firmware_size))
        {
            /* after upgrade the app clear the flag */
            SrvUpgradeObj.FirmwareInfo.update = false;
            SrvUpgradeObj.FirmwareInfo.compelet = false;
            SrvUpgradeObj.FirmwareInfo.firmware_size = 0;

            Storage.update(PARA_TYPE, p_search->item.data_addr, (uint8_t *)&SrvUpgradeObj.FirmwareInfo, sizeof(SrvUpgrade_BaseInfo_TypeDef));
        }
    }
#endif

    SrvUpgradeObj.init_state = true;
    SrvUpgradeObj.send = tx_cb;

    return true;
}

/* copy firmware from back up area to stm32h743 internal flash */
#if (CODE_TYPE == ON_BOOT)
static bool SrvUpgrade_Upgrade_Firmware(uint32_t firmware_size)
{
    uint32_t app_addr = App_Address_Base;
    uint32_t flash_sector_size = 0;

    if (!SrvUpgradeObj.init_state || \
        (SrvUpgradeObj.firmware_buf == NULL) || \
        (BspFlash.erase == NULL) || \
        (BspFlash.write == NULL))
        return false;

    /* update firmware to stm32h743 internal flash */
    if (!StorageFirmware.read_sec(SrvUpgradeObj.firmware_buf, App_Firmware_Size) || \
        !BspFlash.erase(app_addr, App_Section_Size) || \
        !BspFlash.write(app_addr, SrvUpgradeObj.firmware_buf, firmware_size))
        return false;

    /* after update clear temporary buff */
    memset(SrvUpgradeObj.firmware_buf, 0, App_Firmware_Size);

    /* after firmware loaded */
    /* clear all data in storage section */
    return StorageFirmware.erase();
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

static void SrvUpgrade_Firmware_Rec_Start(void *arg, uint8_t *p_data, uint16_t pack_size, uint8_t *p_payload, uint16_t payload_size, bool valid)
{
    char *p_file_name = NULL;
    char *p_file_size = NULL;
    uint16_t offset = 0;
    Storage_ItemSearchOut_TypeDef *p_search = &SrvUpgradeObj.SearchOut;
    
    SrvUpgradeObj.pack_ok_cnt = 0;
    if (valid && (p_payload != NULL))
    {
        /* get file name */
        p_file_name = (char *)p_payload;

        /* get file size */
        offset = strlen(p_file_name) + 1;
        p_file_size = (char *)(p_payload + offset);

        for (uint16_t i = 0; i < strlen(p_file_size); i++)
        {
            if (p_file_size[i] == 0x20)
                p_file_size[i] = '\0';
        }

        SrvUpgradeObj.firmware_name = SrvOsCommon.malloc(strlen(p_file_name) + 1);
        if (SrvUpgradeObj.firmware_name)
        {
            memcpy(SrvUpgradeObj.firmware_name, p_file_name, strlen(p_file_name) + 1);
            SrvUpgradeObj.firmware_size = atoi(p_file_size);

            /* update storage data */
            SrvUpgradeObj.FirmwareInfo.compelet = false;
            SrvUpgradeObj.FirmwareInfo.update = false;
            SrvUpgradeObj.FirmwareInfo.firmware_size = SrvUpgradeObj.firmware_size;

            Storage.update(PARA_TYPE, p_search->item.data_addr, (uint8_t *)&SrvUpgradeObj.FirmwareInfo, sizeof(SrvUpgrade_BaseInfo_TypeDef));
        }
        SrvUpgradeObj.firmware_rec_size = 0;
    }

    Queue.reset(&SrvUpgradeObj.p_queue);
}

static void SrvUpgrade_Firmware_Rec_Pack(void *arg, uint8_t *p_data, uint16_t pack_size, uint8_t *p_payload, uint16_t payload_size, bool valid)
{
    /* update pack into ram */
    if (valid)
    {
        SrvUpgradeObj.pack_ok_cnt ++;
        if (SrvUpgradeObj.firmware_buf)
            memcpy(SrvUpgradeObj.firmware_buf + SrvUpgradeObj.firmware_rec_size, p_payload, payload_size);
        SrvUpgradeObj.firmware_rec_size += payload_size;
    }
    else
    {
        SrvUpgradeObj.pack_err_cnt ++;
    }

    Queue.reset(&SrvUpgradeObj.p_queue);
}

static void SrvUpgrade_Firmware_Rec_EOT(void *arg, uint8_t *p_data, uint16_t size)
{
    Queue.reset(&SrvUpgradeObj.p_queue);
}

static void SrvUpgrade_Firmware_Rec_Done(void *arg, int8_t code)
{
    SrvUpgradeObj.YM_hdl = 0;
    SrvUpgradeObj.pack_ok_cnt = 0;
    Storage_ItemSearchOut_TypeDef *p_search = &SrvUpgradeObj.SearchOut;

    if (code == YModem_Rx_Done)
    {
        /* store firmware into external or on chip flash */
       if (SrvUpgradeObj.send)
       {
            SrvOsCommon.delay_ms(50);
            SrvUpgradeObj.send(arg, (uint8_t *)("YModem finish\r\n"), strlen("YModem finish\r\n"));

            /* print / firmware name / firmware size / receive size / */
       }

       /* check firmware size */
       if (SrvUpgradeObj.firmware_size && \
           SrvUpgradeObj.firmware_rec_size && \
           (SrvUpgradeObj.firmware_rec_size >= SrvUpgradeObj.firmware_size))
       {
            /* update storage data */
            SrvUpgradeObj.FirmwareInfo.compelet = true;
            SrvUpgradeObj.FirmwareInfo.update = true;
            Storage.update(PARA_TYPE, p_search->item.data_addr, (uint8_t *)&SrvUpgradeObj.FirmwareInfo, sizeof(SrvUpgrade_BaseInfo_TypeDef));

            /* update firmware data to flash and storage */
            /* start from erase whole firmware sector on external flash module */
            StorageFirmware.erase();
            StorageFirmware.write_sec(SrvUpgradeObj.firmware_buf, SrvUpgradeObj.firmware_size);

            /* test code */
            // SrvUpgrade_DumpFirmware(From_ERom, arg, SrvUpgradeObj.send); 
            /* test code */

#if (CODE_TYPE == ON_BOOT)
            /* In boot stage, after this step new firmware become effective after reboot directly */
            SrvOsCommon.reboot();
#endif
       }
    }

    if (code == YModem_Rx_Error)
    {
        SrvOsCommon.delay_ms(50);
        SrvUpgradeObj.send(arg, (uint8_t *)("YModem error\r\n"), strlen("YModem error\r\n"));

        memset(SrvUpgradeObj.firmware_name, '\0', strlen((const char *)SrvUpgradeObj.firmware_name));
        memset(SrvUpgradeObj.firmware_buf, 0, SrvUpgradeObj.firmware_rec_size);
        SrvUpgradeObj.firmware_rec_size = 0;
        SrvUpgradeObj.firmware_size = 0;

        SrvUpgradeObj.FirmwareInfo.compelet = false;
        SrvUpgradeObj.FirmwareInfo.update = false;
        SrvUpgradeObj.FirmwareInfo.firmware_size = 0;

        Storage.update(PARA_TYPE, p_search->item.data_addr, (uint8_t *)&SrvUpgradeObj.FirmwareInfo, sizeof(SrvUpgrade_BaseInfo_TypeDef));
        SrvOsCommon.free(SrvUpgradeObj.firmware_name); 
    }

    Queue.reset(&SrvUpgradeObj.p_queue);
    SrvUpgradeObj.mode = Upgrade_Normal_Mode;
}

static bool SrvUpgrade_Firmware_Download(void *com_obj, uint8_t *p_data, uint16_t size)
{
    uint8_t *p_tmp = NULL;
    uint16_t p_size = 0;
    uint32_t update_time = 0;

    if (size)
        update_time = SrvOsCommon.get_os_ms();

#if (CODE_TYPE == ON_BOOT)
    if (SrvUpgradeObj.mode != Upgrade_Force_Mode)
    {
        if (SrvUpgradeObj.send && p_data && size)
            SrvUpgradeObj.send(com_obj, p_data, size);

        return false;
    }
#endif

    if (!SrvUpgradeObj.init_state || (SrvUpgradeObj.send == NULL))
        return false;

    /* Create YMdoem object */
    if (SrvUpgradeObj.YM_hdl == 0)
        SrvUpgradeObj.YM_hdl = YModem.Init(com_obj, YModem_Dir_Rx, SrvOsCommon.malloc, SrvOsCommon.free, SrvUpgradeObj.send, \
                                           SrvUpgrade_Firmware_Rec_Start, SrvUpgrade_Firmware_Rec_EOT, \
                                           SrvUpgrade_Firmware_Rec_Done, SrvUpgrade_Firmware_Rec_Pack);
    
    p_size = Queue.size(SrvUpgradeObj.p_queue);
    if (p_size)
    {
        p_tmp = SrvOsCommon.malloc(p_size);
        
        for (uint16_t i = 0; i < p_size; i++)
        {
            Queue.peek(&SrvUpgradeObj.p_queue, i, &p_tmp[i], 1);
        }
    }

    YModem.Rx(SrvUpgradeObj.YM_hdl, update_time, SrvOsCommon.get_os_ms(), p_tmp, p_size);

    if (p_tmp)
        SrvOsCommon.free(p_tmp);

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

    SrvUpgrade_Firmware_Download(com_obj, p_data, size);

#if (CODE_TYPE == ON_BOOT)
    /* if in boot mode check force code input */
    SrvUpgrade_Check_ForceMode_Enable(com_obj);
#endif

    SrvUpgradeObj.rec_cnt ++;
}

static void SrvUpgrade_Set_TransCallback(SrvUpgrade_Send_Callback tx_cb)
{
    SrvUpgradeObj.send = tx_cb;
}

static bool SrvUpgrade_DumpFirmware(SrvUpgrade_FirmwareDumpType_List type, void *port, SrvUpgrade_Send_Callback tx_cb)
{
    uint8_t *p_data = NULL;
    uint32_t size = 0;
    uint16_t trans_size = FIRMWARE_DUMP_SIZE;

    /* when YM_hdl is set indicate this service is under transmitting */
    if (!SrvUpgradeObj.init_state || SrvUpgradeObj.YM_hdl || (SrvUpgradeObj.FirmwareInfo.firmware_size == 0))
        return false;

    /* test code */
    char *cn[] = {
        "1\r\n",
        "2\r\n",
        "3\r\n",
        "4\r\n",
        "5\r\n",
        "6\r\n",
        "7\r\n",
        "8\r\n",
        "9\r\n",
        "10\r\n",
    };

    for (uint8_t i = 0; i < 10; i++)
    {
        if (tx_cb)
            tx_cb(port, (uint8_t *)cn[i], strlen(cn[i]));

        SrvOsCommon.delay_ms(1000);
    }
    
    SrvOsCommon.delay_ms(1000);
    /* test code */

    /* dump from ram */
    size = SrvUpgradeObj.FirmwareInfo.firmware_size;
    p_data = SrvUpgradeObj.firmware_buf;
    switch ((uint8_t)type)
    {
        case From_Ram:
            break;

        /* dump from external flash chip */
        case From_ERom:
            /* clear temporary buff first */
            memset(SrvUpgradeObj.firmware_buf, 0, size);
            if (!StorageFirmware.read_sec(SrvUpgradeObj.firmware_buf, size))
                return false;
            break;

        /* dump from internal flash still in developping */
        case From_IRom:
            /* clear temporary buff first */
            memset(SrvUpgradeObj.firmware_buf, 0, size);
            return false;

        default: return false;
    }

    while (size)
    {
        if (size < FIRMWARE_DUMP_SIZE)
        {
            trans_size = size;
            size = 0;
        }
        else
            size -= FIRMWARE_DUMP_SIZE;

        if (tx_cb)
            tx_cb(port, p_data, trans_size);

        p_data += trans_size;
        
        /* dump period force to 20Hz */
        SrvOsCommon.delay_ms(50);
    }

    return true;
}
