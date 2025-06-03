 /* 
 * Auther: 8_B!T0
 * WARNING: NOT ALL CIRCUMSTANCES BEEN TESTED
 * 
 * Bref: Use for storage parameter for drone
 *       can create search delete parameter section as user want
 * 
 * NOTICED: STORAGE MODULE ONLY SUPPORT NOR-FLASH
 */
#include "Storage.h"
#include "Srv_OsCommon.h"
#include "Storage_Bus_Port.h"
#include "Storage_Dev_Port.h"

#define STORAGE_VERSION_LEN             3
#define STORAGE_TAG                     "Storage module"
// #define STORAGE_INFO(stage, fmt, ...)   Debug_Print((const char *)STORAGE_TAG, stage, fmt, ##__VA_ARGS__)
#define STORAGE_INFO(stage, fmt, ...)

#define Item_Capacity_Per_Tab           (Storage_TabSize / sizeof(Storage_Item_TypeDef))
#define TabNum                          (Storage_Item_Capacity / (Flash_Storage_TabSize / StorageItem_Size))
#define TabSize                         (TabNum * Flash_Storage_TabSize)

__attribute__((weak)) void* Storage_Malloc(uint32_t size)
{
    return SrvOsCommon.malloc(size);
}

__attribute__((weak)) void Storage_Free(void **ptr)
{
    if ((ptr == NULL) || (*ptr == NULL))
        return;

    SrvOsCommon.free(*ptr);
    *ptr = NULL;
}

/* internal vriable */
Storage_Monitor_TypeDef Storage_Monitor;
static uint8_t page_data_tmp[(Storage_TabSize * 2)] __attribute__((aligned(4))) = {0};
const uint8_t version[STORAGE_VERSION_LEN] = {0, 0, 2};

/* internal function */
static bool Storage_Build_StorageInfo(void);
static bool Storage_Get_StorageInfo(void);
static bool Storage_Format(void);
static bool Storage_UpdateBaseInfo(void);
static bool Storage_Compare_ItemSlot_CRC(const Storage_Item_TypeDef item);
static bool Storage_Comput_ItemSlot_CRC(Storage_Item_TypeDef *p_item);
static Storage_BaseSecInfo_TypeDef* Storage_Get_SecInfo(Storage_FlashInfo_TypeDef *info, Storage_ParaClassType_List class);
static bool Storage_DeleteSingleDataSlot(uint32_t slot_addr, uint8_t *p_data, Storage_BaseSecInfo_TypeDef *p_Sec);
static Storage_ErrorCode_List Storage_FreeSlot_CheckMerge(uint32_t slot_addr, Storage_FreeSlot_TypeDef *slot_info, Storage_BaseSecInfo_TypeDef *p_Sec);
static Link_State_List Storage_Link_FreeSlot(uint32_t front_free_addr, uint32_t behind_free_addr, uint32_t new_free_addr, Storage_FreeSlot_TypeDef *new_free_slot, Storage_BaseSecInfo_TypeDef *p_Sec);
static Storage_ErrorCode_List Storage_ItemSlot_Update(uint32_t tab_addr, uint8_t item_index, Storage_BaseSecInfo_TypeDef *p_Sec, Storage_Item_TypeDef item);
static bool Storage_Clear_Tab(uint32_t addr, uint32_t tab_num);
static bool Storage_Establish_Tab(Storage_ParaClassType_List class);
static bool Storage_Fill_ReserveSec(uint32_t addr);

/* external function */
static bool Storage_Init(StorageDevObj_TypeDef *ExtDev);
static Storage_ItemSearchOut_TypeDef Storage_Search(Storage_ParaClassType_List _class, const char *name);
static Storage_ErrorCode_List Storage_DeleteItem(Storage_ParaClassType_List _class, const char *name);
static Storage_ErrorCode_List Storage_CreateItem(Storage_ParaClassType_List _class, const char *name, uint8_t *p_data, uint16_t size);
static Storage_ErrorCode_List Storage_SlotData_Update(Storage_ParaClassType_List _class, storage_handle data_slot_hdl, uint8_t *p_data, uint16_t size);
static Storage_ErrorCode_List Storage_Get_Data(Storage_ParaClassType_List _class, Storage_Item_TypeDef item, uint8_t *p_data, uint16_t *size);
static Storage_ErrorCode_List Storage_Get_DevInfo(StorageDevObj_TypeDef *info);

static bool Storage_Erase_FirmwareSection(void);
static bool Storage_Read_Firmware(uint8_t *p_data, uint32_t size);
static bool Storage_Write_Firmware(uint8_t *p_data, uint32_t size);

Storage_TypeDef Storage = {
    .init           = Storage_Init,
    .search         = Storage_Search,
    .create         = Storage_CreateItem,
    .remove         = Storage_DeleteItem,
    .get            = Storage_Get_Data,
    .update         = Storage_SlotData_Update,
    .get_dev_info   = Storage_Get_DevInfo,
};

StorageFirmware_TypeDef StorageFirmware = {
    .erase     = Storage_Erase_FirmwareSection,
    .read_sec  = Storage_Read_Firmware,
    .write_sec = Storage_Write_Firmware,
};

static bool Storage_Init(StorageDevObj_TypeDef *ExtDev)
{
    void *bus_cfg = NULL;
    memset(&Storage_Monitor, 0, sizeof(Storage_Monitor));

    Storage_Monitor.init_state = false;

    /* external flash init */
    if (ExtDev == NULL)
        return false;

    Storage_Monitor.ExtDev_ptr = NULL;
    /* doing bus init on your hardware platform */
    bus_cfg = StoragePort_Api.init(Storage_Malloc, Storage_Free);
    if (bus_cfg == NULL)
    {
        STORAGE_INFO("Bus Init", "Failed");
        Storage_Monitor.Flash_Error_Code = Storage_BusInit_Error;
        return false;
    }

    STORAGE_INFO("Bus init", "accomplished");
    STORAGE_INFO("", "Data Slot size %d", sizeof(Storage_DataSlot_TypeDef));
    Storage_Monitor.ExtBusCfg_Ptr = bus_cfg;

    if (ExtDev->chip_type >= Storage_ChipType_All)
    {
        STORAGE_INFO("init", "Unknown chip type");
        Storage_Monitor.Flash_Error_Code = Storage_ModuleType_Error;
        return false;
    }

    if (!StorageDev.set(ExtDev))
    {
        STORAGE_INFO("init", "Device set error");
        Storage_Monitor.Flash_Error_Code = Storage_ExtDevObj_Error;
        return false;
    }

    Storage_Monitor.ExtDev_ptr = ExtDev;
    Storage_Monitor.Flash_ReInit_cnt = Module_ReInit_Cnt;

reinit_external_flash_module:
    Storage_Monitor.Flash_Error_Code = Storage_Error_None;
    if (!StorageDev.init(ExtDev, &Storage_Monitor.module_prod_code))
    {
        Storage_Monitor.Flash_Error_Code = Storage_ModuleInit_Error;
        STORAGE_INFO("init", "Failed");
        if (Storage_Monitor.Flash_ReInit_cnt)
        {
            STORAGE_INFO("init", "Retry remain %d", Storage_Monitor.Flash_ReInit_cnt);
            Storage_Monitor.Flash_ReInit_cnt --;
            goto reinit_external_flash_module;
        }
        return false;
    }

    /* set external flash device read write base address */
    Storage_Monitor.info.phy_start_addr = ExtDev->start_addr;
    Storage_Monitor.info.base_addr = Storage_Start_Addr + ExtDev->start_addr;
    Storage_Monitor.Flash_Format_cnt = Format_Retry_Cnt;

    /* check storage item size */
    if (StorageItem_Size % StorageItem_Align_Unit)
    {
        STORAGE_INFO("init", "Item size error");
        Storage_Assert(true);
        return false;
    }

reupdate_external_flash_info:
    /* get storage info */
    STORAGE_INFO("init", "Get info");
    if (!Storage_Get_StorageInfo())
    {
reformat_external_flash_info:
        if (Storage_Monitor.Flash_Format_cnt)
        {
            /* format storage device */
            if (!Storage_Format())
            {
                Storage_Monitor.Flash_Format_cnt --;
                Storage_Monitor.info.base_addr = Storage_Start_Addr + ExtDev->start_addr;
                if (Storage_Monitor.Flash_Format_cnt == 0)
                {
                    STORAGE_INFO("init", "Format failed");
                    return false;
                }

                goto reformat_external_flash_info;
            }

            /* external flash module format successed */
            /* build storage tab */
            STORAGE_INFO("init", "Build info");
            if (Storage_Build_StorageInfo())
            {
                Storage_Monitor.Flash_BuildTab_cnt ++;
                Storage_Monitor.init_state = true;

                /* after tab builded read storage info again */
                goto reupdate_external_flash_info;
            }
        }
    }
    else
    {
        STORAGE_INFO("chip init", "done");
        Storage_Monitor.init_state = true;
    }

    return Storage_Monitor.init_state;
}

static bool Storage_UpdateBaseInfo(void)
{
    uint16_t crc = 0;

    /* update base info CRC */
    memset(page_data_tmp, 0, Storage_InfoPageSize);
    memcpy(page_data_tmp, &Storage_Monitor.info, sizeof(Storage_FlashInfo_TypeDef));
    crc = Common_CRC16(page_data_tmp, Storage_InfoPageSize - sizeof(crc));
    memcpy(&page_data_tmp[Storage_InfoPageSize - sizeof(crc)], &crc, sizeof(crc));

    if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, page_data_tmp, Storage_TabSize))
        return false;

    return true;
}

static Storage_ErrorCode_List Storage_Get_DevInfo(StorageDevObj_TypeDef *info)
{
    StorageDevObj_TypeDef *p_dev = NULL;
    
    if ((info == NULL) || \
        !Storage_Monitor.init_state || \
        (Storage_Monitor.ExtDev_ptr == NULL))
        return Storage_ExtDevObj_Error;

    memset(info, 0, sizeof(StorageDevObj_TypeDef));
    p_dev = To_StorageDevObj_Ptr(Storage_Monitor.ExtDev_ptr);
    memcpy(info, p_dev, sizeof(StorageDevObj_TypeDef));
    return Storage_Error_None;
}

static bool Storage_Format(void)
{
    uint32_t size = 0;
    uint8_t default_data = 0;
    uint32_t read_time = 0;
    uint32_t remain_size = 0;
    uint32_t addr_tmp = Storage_Monitor.info.base_addr;
        
    size = Storage_TabSize;
    default_data = Flash_Storage_DefaultData;
    remain_size = Flash_Storage_TotalSize;
    read_time = Flash_Storage_TotalSize / Storage_TabSize;
    if (Flash_Storage_TotalSize % Storage_TabSize)
        read_time ++;

    STORAGE_INFO("format", "Start");
    for(uint32_t i = 0; i < read_time; i++)
    {
        if ((remain_size != 0) && (remain_size < size))
            size = remain_size;

        if (!StorageDev.param_erase(Storage_Monitor.ExtDev_ptr, addr_tmp, size) || \
            !StorageDev.param_read(Storage_Monitor.ExtDev_ptr, addr_tmp, page_data_tmp, size))
        {
            STORAGE_INFO("format", "Failed at addr 0x%08x", addr_tmp);
            Storage_Assert(true);
            return false;
        }

        for(uint32_t j = 0; j < size; j++)
        {
            if (page_data_tmp[j] != default_data)
            {
                STORAGE_INFO("format", "Default data error");
                Storage_Assert(true);
                return false;
            }
        }

        addr_tmp += size;
        remain_size -= size;

        if (remain_size == 0)
        {
            STORAGE_INFO("format", "Done");
            return true;
        }
    }
    
    STORAGE_INFO("format", "Failed");
    return false;
}

static bool Storage_Check_Tab(Storage_BaseSecInfo_TypeDef *sec_info)
{
    uint32_t tab_addr = 0;
    uint32_t store_param_found = 0;
    uint32_t store_param_size = 0;
    uint32_t free_slot_addr = 0;
    uint32_t sec_start_addr = 0;
    uint32_t sec_end_addr = 0;
    uint16_t crc16 = 0;
    uint8_t *crc_buf = NULL;
    uint16_t crc_len = 0;
    Storage_FreeSlot_TypeDef FreeSlot_Info;
    Storage_Item_TypeDef *p_ItemList = NULL;

    if (sec_info == NULL)
        return false;

    /* free address & slot check */
    free_slot_addr = sec_info->free_slot_addr;
    sec_start_addr = sec_info->data_sec_addr;
    sec_end_addr = sec_start_addr + sec_info->data_sec_size;

    for(uint32_t free_i = 0; ;)
    {
        STORAGE_INFO("check tab", "freeslot %d addr 0x%08X", free_i, free_slot_addr);
        /* check free slot */
        if ((free_slot_addr == 0) || \
            (free_slot_addr < sec_start_addr) || \
            (free_slot_addr > sec_end_addr) || \
            !StorageDev.param_read(Storage_Monitor.ExtDev_ptr, free_slot_addr, (uint8_t *)&FreeSlot_Info, sizeof(Storage_FreeSlot_TypeDef)))
        {
            if ((free_slot_addr < sec_start_addr) || \
                (free_slot_addr > sec_end_addr))
            {
                STORAGE_INFO("check tab", "free slot addr warnning");
                STORAGE_INFO("check tab", "sec start addr %d", sec_start_addr);
                STORAGE_INFO("check tab", "sec end   addr %d", sec_end_addr);
                STORAGE_INFO("check tab", "free slot addr %d", free_slot_addr);
            }

            break;
        }

        STORAGE_INFO("check tab", "freeslot %d addr 0x%08X", free_i, free_slot_addr);

        if ((FreeSlot_Info.head_tag != STORAGE_SLOT_HEAD_TAG) || \
            (FreeSlot_Info.end_tag != STORAGE_SLOT_END_TAG))
        {
            STORAGE_INFO("check tab", "free slot tag error");
            return false;
        }

        free_i ++;
        if (FreeSlot_Info.nxt_addr == 0)
            break;

        free_slot_addr = FreeSlot_Info.nxt_addr;
    }
    
    if (sec_info->para_num)
    {
        tab_addr = sec_info->tab_addr;

        for (uint16_t tab_i = 0; tab_i < sec_info->tab_num; tab_i ++)
        {
            if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, tab_addr, page_data_tmp, sec_info->tab_size / sec_info->tab_num))
                return false;
        
            p_ItemList = (Storage_Item_TypeDef *)page_data_tmp;
            for(uint16_t item_i = 0; item_i < Item_Capacity_Per_Tab; item_i ++)
            {
                if ((p_ItemList[item_i].head_tag == STORAGE_ITEM_HEAD_TAG) && \
                    (p_ItemList[item_i].end_tag == STORAGE_ITEM_END_TAG))
                {
                    /*  check item slot crc
                     *  typedef struct
                     *  {
                     *      uint8_t head_tag;
                     *      uint8_t class;
                     *      uint8_t name[STORAGE_NAME_LEN];
                     *      uint32_t data_addr;
                     *      uint16_t len;
                     *      uint16_t crc16;
                     *      uint8_t end_tag;
                     *  } Storage_Item_TypeDef;
                     *  
                     *  comput crc from class to len */
                    crc_buf = (uint8_t *)&p_ItemList[item_i] + sizeof(p_ItemList[item_i].head_tag);
                    crc_len = sizeof(Storage_Item_TypeDef);
                    crc_len -= sizeof(p_ItemList[item_i].head_tag);
                    crc_len -= sizeof(p_ItemList[item_i].end_tag);
                    crc_len -= sizeof(p_ItemList[item_i].crc16);
                    
                    crc16 = Common_CRC16(crc_buf, crc_len);
                    if (crc16 != p_ItemList[item_i].crc16)
                    {
                        STORAGE_INFO("check tab", "item[%d] %s crc error", item_i, p_ItemList[item_i].name);
                        return false;
                    }
                    
                    if (memcmp(p_ItemList[item_i].name, STORAGE_FREEITEM_NAME, strlen(STORAGE_FREEITEM_NAME)) != 0)
                    {
                        store_param_found ++;
                        store_param_size += p_ItemList[item_i].len;
                    }
                }
            }

            tab_addr += (sec_info->tab_size / sec_info->tab_num);
        }

        if ((store_param_found != sec_info->para_num) || \
            (store_param_size != sec_info->para_size))
        {
            STORAGE_INFO("check tab", "para num or size error");
            return false;
        }
    }

    return true;
}

static bool Storage_Get_StorageInfo(void)
{
    bool tab_state = true;
    Storage_FlashInfo_TypeDef *p_Info = NULL;
    Storage_FlashInfo_TypeDef Info_r;
    uint16_t crc = 0;
    uint16_t crc_read = 0;
    char flash_tag[EXTERNAL_PAGE_TAG_SIZE];

    memset(flash_tag, '\0', sizeof(flash_tag));
    memset(&Info_r, 0, sizeof(Storage_FlashInfo_TypeDef));

    memcpy(flash_tag, EXTERNAL_STORAGE_PAGE_TAG, EXTERNAL_PAGE_TAG_SIZE);
    p_Info = &Storage_Monitor.info;
    
    if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, page_data_tmp, Storage_TabSize))
    {
        STORAGE_INFO("read", "addr 0x%08X %d size data failed", Storage_Monitor.info.base_addr, Storage_TabSize);
        return false;
    }

    /* check internal storage tag */
    Info_r = *(Storage_FlashInfo_TypeDef *)page_data_tmp;
    
    /* check storage tag */
    /* check boot / sys / user  start addr */
    if ((strcmp((const char *)Info_r.tag, flash_tag) != 0) || \
        (Info_r.sys_sec.tab_addr == 0) || \
        (Info_r.user_sec.tab_addr == 0) || \
        (Info_r.sys_sec.tab_addr == Info_r.user_sec.tab_addr))
        return false;
    
    STORAGE_INFO("info", "tag %s", Info_r.tag);
    STORAGE_INFO("info", "sys  tab addr 0x%08X", Info_r.sys_sec.tab_addr);
    STORAGE_INFO("info", "user tab addr 0x%08X", Info_r.user_sec.tab_addr);

    /* get crc from storage baseinfo section check crc value */
    memcpy(&crc_read, &page_data_tmp[Storage_InfoPageSize - sizeof(uint16_t)], sizeof(uint16_t));
    crc = Common_CRC16(page_data_tmp, Storage_InfoPageSize - sizeof(crc));
    if (crc != crc_read)
    {
        STORAGE_INFO("info sec", "CRC Error");
        return false;
    }

    memset(page_data_tmp, 0, Storage_TabSize);
    /* check system section tab & free slot info & stored item */
    /* check  user  section tab & free slot info & stored item */
    STORAGE_INFO("info", "check sys tab");
    if (!Storage_Check_Tab(&Info_r.sys_sec))
    {
        tab_state = false;
        STORAGE_INFO("info", "sys tab error");
    }
    
    STORAGE_INFO("info", "check user tab");
    if (!Storage_Check_Tab(&Info_r.user_sec))
    {
        tab_state = false;
        STORAGE_INFO("info", "user tab error");
    }

    if (tab_state)
    {
        STORAGE_INFO("info", "all tab checked");
        memcpy(p_Info, &Info_r, sizeof(Storage_FlashInfo_TypeDef));
        return true;
    }

    return false;
}

/*
 * Write 0 to table area 
 */
static bool Storage_Clear_Tab(uint32_t addr, uint32_t tab_num)
{
    uint32_t addr_tmp = 0;

    if ((addr == 0) || \
        (tab_num == 0))
        return false;

    memset(page_data_tmp, 0, Storage_TabSize);
    addr_tmp = addr;
    
    for(uint32_t i = 0; i < tab_num; i++)
    {
        if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, addr_tmp, page_data_tmp, Storage_TabSize))
            return false;

        addr_tmp += Storage_TabSize;
    }

    return true;
}

/* 
 * if matched return data slot address 
 * else return 0
 */
static Storage_ItemSearchOut_TypeDef Storage_Search(Storage_ParaClassType_List _class, const char *name)
{
    Storage_ItemSearchOut_TypeDef ItemSearch;
    Storage_BaseSecInfo_TypeDef *p_Sec = NULL;
    Storage_Item_TypeDef *item_list = NULL;
    Storage_Item_TypeDef *p_item = NULL;
    uint32_t tab_addr = 0;

    memset(&ItemSearch, 0, sizeof(Storage_ItemSearchOut_TypeDef));

    if (!Storage_Monitor.init_state || \
        (name == NULL) || \
        (strlen(name) == 0) || \
        (_class > Para_User))
        return ItemSearch;

    p_Sec = Storage_Get_SecInfo(&Storage_Monitor.info, _class);

    if ((p_Sec == NULL) || \
        (p_Sec->para_num == 0) || \
        (p_Sec->para_size == 0))
        return ItemSearch;

    tab_addr = p_Sec->tab_addr;
    /* tab traverse */
    for (uint8_t tab_i = 0; tab_i < p_Sec->tab_num; tab_i ++)
    {
        if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, tab_addr, page_data_tmp, (p_Sec->tab_size / p_Sec->tab_num)))
            return ItemSearch;
    
        /* tab item traverse */
        item_list = (Storage_Item_TypeDef *)page_data_tmp;
        for (uint16_t item_i = 0; item_i < ((p_Sec->tab_size / p_Sec->tab_num) / sizeof(Storage_Item_TypeDef)); item_i ++)
        {
            p_item = &item_list[item_i];

            if ((p_item->head_tag == STORAGE_ITEM_HEAD_TAG) && \
                (p_item->end_tag == STORAGE_ITEM_END_TAG) && \
                (memcmp(p_item->name, name, strlen(name)) == 0) && \
                (Storage_Compare_ItemSlot_CRC(*p_item)))
            {
                ItemSearch.item_addr = tab_addr;
                ItemSearch.item_index = item_i;
                ItemSearch.item = *p_item;
                return ItemSearch;
            }
        }
    
        /* update tab address */
        tab_addr += (p_Sec->tab_size / p_Sec->tab_num);
    }

    return ItemSearch;
}

static Storage_ErrorCode_List Storage_ItemSlot_Update(uint32_t tab_addr, uint8_t item_index, Storage_BaseSecInfo_TypeDef *p_Sec, Storage_Item_TypeDef item)
{
    Storage_Item_TypeDef *ItemList = NULL;

    if ((tab_addr == 0) || \
        (p_Sec == NULL) || \
        (item.head_tag != STORAGE_ITEM_HEAD_TAG) || \
        (item.end_tag != STORAGE_ITEM_END_TAG))
        return Storage_Param_Error;

    if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, tab_addr, &page_data_tmp[Storage_TabSize], Storage_TabSize))
        return Storage_Read_Error;

    ItemList = (Storage_Item_TypeDef *)&page_data_tmp[Storage_TabSize];
    ItemList[item_index] = item;

    if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, tab_addr, (uint8_t *)ItemList, Storage_TabSize))
        return Storage_Write_Error;

    return Storage_Error_None;
}

static Storage_ErrorCode_List Storage_Get_Data(Storage_ParaClassType_List class, Storage_Item_TypeDef item, uint8_t *p_data, uint16_t *size)
{
    int32_t data_len = 0;
    uint16_t valid_size = 0;
    uint32_t data_addr = 0;
    uint8_t *p_read_out = NULL;
    uint8_t *crc_buf = NULL;
    uint16_t crc_len = 0;
    uint16_t crc = 0;
    Storage_DataSlot_TypeDef DataSlot;
    uint8_t *p_data_start = NULL;

    memset(&DataSlot, 0, sizeof(Storage_DataSlot_TypeDef));
    if (item.data_addr && p_data)
    {
        data_len = item.len;
        data_addr = item.data_addr;

        while(data_len)
        {
            if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, data_addr, page_data_tmp, data_len + sizeof(Storage_DataSlot_TypeDef)))
                return Storage_GetData_Error;

            p_read_out = page_data_tmp;
            memcpy(&DataSlot.head_tag, p_read_out, sizeof(DataSlot.head_tag));
            p_read_out += sizeof(DataSlot.head_tag);
            if (DataSlot.head_tag != STORAGE_SLOT_HEAD_TAG)
                return Storage_GetData_Error;

            memcpy(&DataSlot.total_data_size, p_read_out, sizeof(DataSlot.total_data_size));
            p_read_out += sizeof(DataSlot.total_data_size);
            if (DataSlot.total_data_size == 0)
                return Storage_GetData_Error;

            memcpy(&DataSlot.cur_slot_size, p_read_out, sizeof(DataSlot.cur_slot_size));
            p_read_out += sizeof(DataSlot.cur_slot_size);
            if (DataSlot.cur_slot_size == 0)
                return Storage_GetData_Error;

            memcpy(&DataSlot.nxt_addr, p_read_out, sizeof(DataSlot.nxt_addr));
            p_read_out += sizeof(DataSlot.nxt_addr);

            memcpy(&DataSlot.frag_len, p_read_out, sizeof(DataSlot.frag_len));
            p_read_out += sizeof(DataSlot.frag_len);

            memcpy(&DataSlot.align_size, p_read_out, sizeof(DataSlot.align_size));
            p_read_out += sizeof(DataSlot.align_size);
            p_data_start = p_read_out;

            crc_buf = p_read_out;
            crc_len = DataSlot.cur_slot_size;
            crc = Common_CRC16(crc_buf, crc_len);

            p_read_out += DataSlot.cur_slot_size;
            memcpy(&DataSlot.slot_crc, p_read_out, sizeof(DataSlot.slot_crc));
            p_read_out += sizeof(DataSlot.slot_crc);
            if (crc != DataSlot.slot_crc)
                return Storage_GetData_Error;
            
            memcpy(&DataSlot.res, p_read_out, sizeof(DataSlot.res));
            p_read_out += sizeof(DataSlot.res);

            memcpy(&DataSlot.end_tag, p_read_out, sizeof(DataSlot.end_tag));
            if (DataSlot.end_tag != STORAGE_SLOT_END_TAG)
                return Storage_GetData_Error;

            memcpy(p_data, p_data_start, DataSlot.cur_slot_size - DataSlot.align_size);
            data_len -= DataSlot.cur_slot_size;
            valid_size += DataSlot.cur_slot_size - DataSlot.align_size;

            if (data_len < 0)
                return Storage_GetData_Error;

            if (DataSlot.nxt_addr)
            {
                p_data += DataSlot.cur_slot_size - DataSlot.align_size;
                data_addr = DataSlot.nxt_addr;
            }
            else
            {
                if (data_len == 0)
                    break;

                return Storage_GetData_Error;
            }
        }

        if (size)
            *size = valid_size;

        return Storage_Error_None;
    }
    
    return Storage_GetData_Error;
}

static Storage_ErrorCode_List Storage_SlotData_Update(Storage_ParaClassType_List _class, storage_handle data_slot_hdl, uint8_t *p_data, uint16_t size)
{
    Storage_BaseSecInfo_TypeDef *p_Sec = NULL;
    Storage_DataSlot_TypeDef *p_slotdata = NULL;
    uint8_t *p_read_tmp = page_data_tmp;
    uint8_t *crc_buf = NULL;
    uint16_t crc = 0;
    uint32_t read_addr = 0;
    uint32_t read_size = 0;
    uint32_t valid_data_size = 0;
    uint8_t align_byte = 0;

    if (!Storage_Monitor.init_state || \
        (_class > Para_User) || \
        (data_slot_hdl == 0) || \
        (p_data == NULL) || \
        (size == 0))
        return Storage_Param_Error;

    p_Sec = Storage_Get_SecInfo(&Storage_Monitor.info, _class);
    
    if ((p_Sec == NULL) || \
        (p_Sec->data_sec_addr > data_slot_hdl) || \
        (data_slot_hdl > (p_Sec->data_sec_addr + p_Sec->data_sec_size)))
        return Storage_Param_Error;

    read_addr = data_slot_hdl;
    memset(page_data_tmp, 0, sizeof(Storage_DataSlot_TypeDef));
    if (size % STORAGE_DATA_ALIGN)
        align_byte = STORAGE_DATA_ALIGN - size % STORAGE_DATA_ALIGN;

    /* get data slot first */
    if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, read_addr, p_read_tmp, sizeof(Storage_DataSlot_TypeDef)))
        return Storage_Read_Error;
    
    /* get data size */
    p_slotdata = (Storage_DataSlot_TypeDef *)p_read_tmp;
    if ((p_slotdata->head_tag != STORAGE_SLOT_HEAD_TAG) || \
        (p_slotdata->total_data_size == 0) || \
        (p_slotdata->total_data_size > p_Sec->data_sec_size))
        return Storage_DataInfo_Error;

    if (p_slotdata->total_data_size != (size + align_byte))
        return Storage_Update_DataSize_Error;

    read_size = p_slotdata->total_data_size + sizeof(Storage_DataSlot_TypeDef);
    p_read_tmp = page_data_tmp;
    memset(p_read_tmp, 0, sizeof(Storage_DataSlot_TypeDef));
    p_slotdata = NULL;
    while(true)
    {
        p_slotdata = (Storage_DataSlot_TypeDef *)p_read_tmp;

        /* get data from handle */
        if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, read_addr, p_read_tmp, read_size))
            return Storage_Read_Error;
        
        p_slotdata->head_tag = *((uint32_t *)p_read_tmp);
        p_read_tmp += sizeof(p_slotdata->head_tag);
        if (p_slotdata->head_tag != STORAGE_SLOT_HEAD_TAG)
            return Storage_DataInfo_Error;

        p_slotdata->total_data_size = *((uint32_t *)p_read_tmp);
        p_read_tmp += sizeof(p_slotdata->total_data_size);
        if ((p_slotdata->total_data_size == 0) || \
            (p_slotdata->total_data_size > p_Sec->data_sec_size))
            return Storage_DataInfo_Error;

        p_slotdata->cur_slot_size = *((uint32_t *)p_read_tmp);
        p_read_tmp += sizeof(p_slotdata->cur_slot_size);
        if (p_slotdata->cur_slot_size > p_slotdata->total_data_size)
            return Storage_DataInfo_Error;

        p_slotdata->nxt_addr = *((uint32_t *)p_read_tmp);
        p_read_tmp += sizeof(p_slotdata->nxt_addr);

        p_slotdata->frag_len = *((uint8_t *)p_read_tmp);
        p_read_tmp += sizeof(p_slotdata->frag_len);

        p_slotdata->align_size = *((uint8_t *)p_read_tmp);
        p_read_tmp += sizeof(p_slotdata->align_size);
        if (p_slotdata->align_size >= STORAGE_DATA_ALIGN)
            return Storage_DataInfo_Error;

        crc_buf = p_read_tmp;
        memcpy(p_read_tmp, p_data, (p_slotdata->cur_slot_size - p_slotdata->align_size));
        p_read_tmp += p_slotdata->cur_slot_size - p_slotdata->align_size;

        if (p_slotdata->align_size)
            /* set align byte */
            memset(p_read_tmp, 0, p_slotdata->align_size);
        
        p_read_tmp += p_slotdata->align_size;
        /* update crc */
        crc = Common_CRC16(crc_buf, p_slotdata->cur_slot_size) ;
        
        valid_data_size += p_slotdata->cur_slot_size - p_slotdata->align_size;
        if (p_slotdata->nxt_addr == 0)
        {
            /* all data has been read out from the data slot in destination section */
            /* compare update data size and valid_data_size */
            if (size != valid_data_size)
                return Storage_Update_DataSize_Error;
        }
        else
        {
            if ((p_slotdata->nxt_addr < p_Sec->data_sec_addr) || \
                (p_slotdata->nxt_addr > p_Sec->data_sec_addr + p_Sec->data_sec_size))
                return Storage_DataInfo_Error; 
        }

        memcpy(p_read_tmp, &crc, sizeof(crc));
        
        p_data += p_slotdata->cur_slot_size - p_slotdata->align_size;
        p_read_tmp += sizeof(p_slotdata->slot_crc);

        p_read_tmp += sizeof(p_slotdata->res);

        if (*(uint32_t *)p_read_tmp != STORAGE_SLOT_END_TAG)
            return Storage_DataInfo_Error;

        if ((p_slotdata->nxt_addr == 0) && \
            (size == valid_data_size))
        {
            /* update data to flash */
            if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, read_addr, page_data_tmp, p_slotdata->cur_slot_size + sizeof(Storage_DataSlot_TypeDef)))
                return Storage_Write_Error;

            /* update accomplish */
            return Storage_Error_None;
        }

        read_addr = p_slotdata->nxt_addr;
    }

    return Storage_Error_None;
}

/* BUG INSIDE */
static Link_State_List Storage_Link_FreeSlot(uint32_t front_free_addr, uint32_t behind_free_addr, uint32_t new_free_addr, Storage_FreeSlot_TypeDef *new_free_slot, Storage_BaseSecInfo_TypeDef *p_Sec)
{
    Storage_FreeSlot_TypeDef front_slot;
    Storage_FreeSlot_TypeDef behind_slot;

    STORAGE_INFO("link free slot", "new    free slot addr 0x%08X", new_free_addr);
    STORAGE_INFO("link free slot", "new    free slot size %d",     new_free_slot->size);
    STORAGE_INFO("link free slot", "front  free slot addr 0x%08X", front_free_addr);
    STORAGE_INFO("link free slot", "behind free slot addr 0x%08X", behind_free_addr);

    if ((new_free_addr == 0) || \
        (new_free_slot == NULL) || \
        (front_free_addr == 0) || \
        (front_free_addr == new_free_addr) || \
        (p_Sec == NULL))
        return Link_Failed;

    memset(&front_slot, 0, sizeof(Storage_FreeSlot_TypeDef));
    memset(&behind_slot, 0, sizeof(Storage_FreeSlot_TypeDef));

    STORAGE_INFO("link free slot", "Linking free slot");

/* 
 *
 *       address N                    address X                   address Y
 * _____________________        _____________________        ______________________
 * |  front free slot  |   ———→ |   new free slot   |   ———→ |  behind free slot  |   ———→ ... ...
 * |                   |   |    |                   |   |    |                    |   |
 * |____next_addr_X____|   |    |____next_addr_Y____|   |    |_____next_addr_Z____|   |
 *          |______________|             |______________|              |______________|
 * 
 */
    /* get front free slot info */
    if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, front_free_addr, (uint8_t *)&front_slot, sizeof(Storage_FreeSlot_TypeDef)) || \
        (front_slot.head_tag != STORAGE_SLOT_HEAD_TAG) || (front_slot.end_tag != STORAGE_SLOT_END_TAG))
        return Link_Failed;

    if ((new_free_addr + new_free_slot->size) <= front_free_addr)
    {
        STORAGE_INFO("link", "In front of the old free slot");
        
        /* new free slot in front of the old free slot */
        if ((new_free_addr + new_free_slot->size) < front_free_addr)
        {
            new_free_slot->nxt_addr = front_free_addr;
        }
        else
        {
            STORAGE_INFO("link", "Merge with the front");
            new_free_slot->size += front_slot.size;
            new_free_slot->nxt_addr = front_slot.nxt_addr;
            memset(&front_slot, 0, sizeof(front_slot));

            if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, front_free_addr, (uint8_t *)&front_slot, sizeof(Storage_FreeSlot_TypeDef)))
                return Link_Failed;
        }

        if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, new_free_addr, (uint8_t *)new_free_slot, sizeof(Storage_FreeSlot_TypeDef)))
            return Link_Failed;
        
        p_Sec->free_slot_addr = new_free_addr;
    }

    if (behind_free_addr != 0)
    {
        STORAGE_INFO("link", "Check behind free slot");

        /* get behind free slot info */
        if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, behind_free_addr, (uint8_t *)&behind_slot, sizeof(Storage_FreeSlot_TypeDef)) || \
            (behind_slot.head_tag != STORAGE_SLOT_HEAD_TAG) || (behind_slot.end_tag != STORAGE_SLOT_END_TAG))
            return Link_Failed;

        if ((behind_free_addr == new_free_addr) || ((new_free_slot->size + new_free_addr) > behind_free_addr))
            return Link_Failed;

        if (behind_free_addr < new_free_addr)
            return Link_Searching;
        
        new_free_slot->nxt_addr = behind_free_addr;
        if ((new_free_addr + new_free_slot->size) == behind_free_addr)
        {
            STORAGE_INFO("link", "Merge with the behind");

            new_free_slot->nxt_addr = behind_slot.nxt_addr;
            new_free_slot->size += behind_slot.size;
            
            memset(&behind_slot, 0, sizeof(Storage_FreeSlot_TypeDef));
            STORAGE_INFO("link", "Update behind slot addr 0x%08X", behind_free_addr);
            if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, behind_free_addr, (uint8_t *)&behind_slot, sizeof(Storage_FreeSlot_TypeDef)))
                return Link_Failed;
            
            if (behind_slot.nxt_addr == 0)
                behind_free_addr = 0;
        }

        if ((front_slot.size + front_free_addr) != new_free_addr)
        {
            front_slot.nxt_addr = new_free_addr;
            new_free_slot->nxt_addr = behind_free_addr;
        }
        else
        {
            STORAGE_INFO("link", "Merge with the front");
            front_slot.nxt_addr = new_free_slot->nxt_addr;
            front_slot.size += new_free_slot->size;
            memset(new_free_slot, 0, sizeof(Storage_FreeSlot_TypeDef));
        }

        if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, front_free_addr, (uint8_t *)&front_slot, sizeof(Storage_FreeSlot_TypeDef)))
            return Link_Failed;
    }
    
    if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, new_free_addr, (uint8_t *)new_free_slot, sizeof(Storage_FreeSlot_TypeDef)))
        return Link_Failed;

    return Link_Done;
}

static Storage_ErrorCode_List Storage_FreeSlot_CheckMerge(uint32_t slot_addr, Storage_FreeSlot_TypeDef *new_freeslot, Storage_BaseSecInfo_TypeDef *p_Sec)
{
    Storage_FreeSlot_TypeDef FreeSlot_Info;
    uint32_t freeslot_addr = 0;
    uint32_t nxt_freeslot_addr = 0;
    Link_State_List slot_link_state = Link_Done;

    if ((p_Sec == NULL) || \
        (new_freeslot == NULL) || \
        (slot_addr < p_Sec->data_sec_addr) || \
        (slot_addr > (p_Sec->data_sec_addr + p_Sec->data_sec_size)))
        return Storage_Param_Error;

    memset(&FreeSlot_Info, 0, sizeof(FreeSlot_Info));

    if ((new_freeslot->head_tag != STORAGE_SLOT_HEAD_TAG) || \
        (new_freeslot->end_tag != STORAGE_SLOT_END_TAG) || \
        (new_freeslot->size > p_Sec->free_space_size))
    {
        STORAGE_INFO("delete", "check merge error");
        if (new_freeslot->head_tag != STORAGE_SLOT_HEAD_TAG)
            STORAGE_INFO("delete", "bad header");

        if (new_freeslot->end_tag != STORAGE_SLOT_END_TAG)
            STORAGE_INFO("delete", "bad ender");

        return Storage_FreeSlot_Info_Error;
    }

    STORAGE_INFO("delete", "check merge");
    freeslot_addr = p_Sec->free_slot_addr;
    while (true)
    {
        /* traverse all free slot */
        if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, freeslot_addr, (uint8_t *)&FreeSlot_Info, sizeof(FreeSlot_Info)))
            return Storage_Read_Error;

        if ((FreeSlot_Info.head_tag != STORAGE_SLOT_HEAD_TAG) || \
            (FreeSlot_Info.end_tag != STORAGE_SLOT_END_TAG))
        {
            STORAGE_INFO("slot merge", "Free slot header or ender error %s", __LINE__);
            return Storage_FreeSlot_Info_Error;
        }

        nxt_freeslot_addr = FreeSlot_Info.nxt_addr;
        STORAGE_INFO("new freeslot", "head  tag: 0x%08X",   new_freeslot->head_tag);
        STORAGE_INFO("new freeslot", "end   tag: 0x%08X",   new_freeslot->end_tag);
        STORAGE_INFO("new freeslot", "slot size: %d",       new_freeslot->size);
        STORAGE_INFO("new freeslot", "next addr: 0x%08X",   new_freeslot->nxt_addr);

        STORAGE_INFO("slot merge", "free slot addr linking");
        slot_link_state = Storage_Link_FreeSlot(freeslot_addr, nxt_freeslot_addr, slot_addr, new_freeslot, p_Sec);

        /* link free slot */
        if (slot_link_state == Link_Failed)
        {
            STORAGE_INFO("slot merge", "free slot link failed");
            return Storage_FreeSlot_Link_Error;
        }
        else if (slot_link_state == Link_Done)
            return Storage_Error_None;

        /* update front free slot address */
        freeslot_addr = nxt_freeslot_addr;
        p_Sec->free_space_size += new_freeslot->size;
    }

    return Storage_Delete_Error;
}

static bool Storage_DeleteSingleDataSlot(uint32_t slot_addr, uint8_t *p_data, Storage_BaseSecInfo_TypeDef *p_Sec)
{
    uint16_t cur_slot_size = 0;
    uint16_t inc_free_space = sizeof(Storage_DataSlot_TypeDef);
    uint8_t *p_freeslot_start = NULL;
    uint8_t *p_freeslot_data = NULL;
    uint8_t *data_w = NULL;

    if ((slot_addr == 0) || \
        (p_Sec == NULL) || \
        (p_data == NULL) || \
        (slot_addr < p_Sec->data_sec_addr) || \
        (slot_addr > (p_Sec->data_sec_addr + p_Sec->data_sec_size)))
        return false;
    
    /* check header */
    if (*((uint32_t *)p_data) != STORAGE_SLOT_HEAD_TAG)
        return false;

    p_freeslot_start = p_data;
    data_w = p_freeslot_start;
    p_data += sizeof(uint32_t);

    /* clear total data size */
    *((uint16_t *)p_data) = 0;
    p_data += sizeof(uint16_t);

    /* get current slot data size */
    cur_slot_size = *((uint16_t *)p_data);
    inc_free_space += cur_slot_size;
    
    /* clear current slot data size */
    *((uint16_t *)p_data) = 0;
    p_data += sizeof(uint16_t);

    /* clear next data slot address */
    *((uint32_t *)p_data) = 0;
    p_data += sizeof(uint32_t);

    /* clear frag len */
    *((uint16_t *)p_data) = 0;
    p_data += sizeof(uint16_t);

    /* clear align size */
    *((uint16_t *)p_data) = 0;
    p_data += sizeof(uint16_t);

    /* clear data */
    memset(p_data, 0, cur_slot_size);
    p_data += cur_slot_size;

    /* clear crc */
    *((uint16_t *)p_data) = 0;
    p_data += sizeof(uint16_t);

    /* clear res */
    *((uint16_t *)p_data) = 0;
    p_data += sizeof(uint16_t);

    /* clear end tag */
    *((uint32_t *)p_data) = 0;

    /* update freeslot data info */
    p_freeslot_data = p_freeslot_start;
    if (*(uint32_t *)p_freeslot_data == STORAGE_SLOT_HEAD_TAG)
    {
        STORAGE_INFO("delete", "update free slot");

        p_freeslot_data += sizeof(uint32_t);

        /* update current free slot size */
        *(uint32_t *)p_freeslot_data = cur_slot_size + sizeof(Storage_DataSlot_TypeDef);
        p_freeslot_data += sizeof(uint32_t);

        /* reset next freeslot addr
         * link free slot address
         *
         * should link free slot
         */
        *(uint32_t *)p_freeslot_data = 0;
        p_freeslot_data += sizeof(uint32_t);

        /* set end tag */
        *(uint32_t *)p_freeslot_data = STORAGE_SLOT_END_TAG;
    }
    else
        return false;

    /* update to data section */
    if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, slot_addr, data_w, inc_free_space))
        return false;

    p_Sec->free_space_size += inc_free_space;

    /* check free slot and merge */
    if (Storage_FreeSlot_CheckMerge(slot_addr, (Storage_FreeSlot_TypeDef *)p_freeslot_start, p_Sec) == Storage_Error_None)
        return true;
    
    return false;
}

static bool Storage_DeleteAllDataSlot(uint32_t addr, char *name, uint32_t total_size, Storage_BaseSecInfo_TypeDef *p_Sec)
{
    Storage_DataSlot_TypeDef data_slot;
    uint8_t *p_read = page_data_tmp;
    uint32_t read_size = sizeof(Storage_DataSlot_TypeDef);

    if ((addr == 0) || \
        (name == NULL) || \
        (strlen(name) == 0) || \
        (total_size == 0) || \
        (p_Sec == NULL))
        return false;

    memset(&data_slot, 0, sizeof(data_slot));
    memset(page_data_tmp, 0, Storage_TabSize);
    if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, addr, page_data_tmp, read_size))
        return false;

    data_slot.head_tag = *((uint32_t *)p_read);
    if (data_slot.head_tag != STORAGE_SLOT_HEAD_TAG)
    {
        STORAGE_INFO("delete", "addr 0x%08x header 0x%08x / 0x%08x error", data_slot.head_tag, STORAGE_SLOT_HEAD_TAG);
        return false;
    }

    p_read += sizeof(data_slot.head_tag);
    
    data_slot.total_data_size = *((uint16_t *)p_read);
    if (data_slot.total_data_size != total_size)
    {
        STORAGE_INFO("delete", "addr 0x%08x total data size %d / %d error", addr, data_slot.total_data_size, total_size);
        return false;
    }

    p_read += sizeof(data_slot.total_data_size);
    data_slot.cur_slot_size = *((uint16_t *)p_read);
    if (data_slot.total_data_size && \
        ((data_slot.cur_slot_size == 0) || \
         (data_slot.cur_slot_size > data_slot.total_data_size)))
    {
        STORAGE_INFO("delete", "addr 0x%08x current slot size %d error", addr, data_slot.cur_slot_size);
        return false;
    }

    /* now have current slot size read data in slot again */
    read_size += data_slot.cur_slot_size;
    if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, addr, page_data_tmp, read_size))
        return false;

    p_read += sizeof(data_slot.cur_slot_size);
    data_slot.nxt_addr = *((uint32_t *)p_read);
    if (data_slot.nxt_addr && \
        ((data_slot.nxt_addr <= p_Sec->data_sec_addr) || \
         (data_slot.nxt_addr >= (p_Sec->data_sec_addr + p_Sec->data_sec_size))))
    {
        STORAGE_INFO("delete", "Next addr 0x%08X error", data_slot.nxt_addr);
        return false;
    }

    p_read += sizeof(data_slot.nxt_addr);
    p_read += sizeof(data_slot.frag_len);

    data_slot.align_size = *((uint8_t *)p_read);    
    if (data_slot.align_size >= STORAGE_DATA_ALIGN)
    {
        STORAGE_INFO("delete", "Align size %d error", data_slot.align_size);
        return false;
    }

    /* clear align size */
    *(uint8_t *)p_read = 0;

    /* delete next slot data */
    if (data_slot.nxt_addr)
    {
        /* traverse slot address */
        if (!Storage_DeleteAllDataSlot(data_slot.nxt_addr, name, total_size, p_Sec))
            return false;
    }

    /* clear data section */
    memset(p_read, 0, data_slot.cur_slot_size);

    /* set current slot data as 0 */
    p_read += sizeof(data_slot.align_size);
    memset(p_read, 0, data_slot.cur_slot_size);
    p_read += data_slot.cur_slot_size;

    /* clear crc */
    *((uint16_t *)p_read) = 0;
    p_read += sizeof(data_slot.slot_crc);

    *((uint16_t *)p_read) = 0;
    p_read += sizeof(data_slot.res);

    /* ender error */
    if (*((uint32_t *)p_read) != STORAGE_SLOT_END_TAG)
    {
        STORAGE_INFO("delete", "Data slot ender error");
        return false;
    }

    /* reset data slot as free slot */
    if (!Storage_DeleteSingleDataSlot(addr, page_data_tmp, p_Sec))
        return false;

    return true;
}

static Storage_ErrorCode_List Storage_DeleteItem(Storage_ParaClassType_List _class, const char *name)
{
    Storage_FlashInfo_TypeDef *p_Flash = NULL;
    Storage_BaseSecInfo_TypeDef *p_Sec = NULL;
    Storage_ItemSearchOut_TypeDef ItemSearch;
    
    memset(&ItemSearch, 0, sizeof(ItemSearch));

    if ( !Storage_Monitor.init_state || \
        (name == NULL) || \
        (strlen(name) == 0) || \
        (strlen(name) >= STORAGE_NAME_LEN))
        return Storage_Param_Error;

    p_Flash = &Storage_Monitor.info;
    p_Sec = Storage_Get_SecInfo(p_Flash, _class);
    if ((p_Sec == NULL) || \
        (p_Sec->para_num == 0) || \
        (p_Sec->para_size == 0))
        return Storage_Class_Error;

    /* search tab for item slot first */
    ItemSearch = Storage_Search(_class, name);
    if ((ItemSearch.item_addr == 0) || \
        (ItemSearch.item.data_addr == 0) || \
        (ItemSearch.item.head_tag != STORAGE_ITEM_HEAD_TAG) || \
        (ItemSearch.item.end_tag != STORAGE_ITEM_END_TAG) || \
        !Storage_DeleteAllDataSlot(ItemSearch.item.data_addr, (char *)name, ItemSearch.item.len, p_Sec))
    {
        STORAGE_INFO("delete slot", "filed");
        return Storage_Delete_Error;
    }

    /* update base info */
    memset(page_data_tmp, 0, Storage_TabSize);
    
    p_Sec->para_num -= 1;
    p_Sec->para_size -= ItemSearch.item.len;

    if (!Storage_UpdateBaseInfo())
        return Storage_Delete_Error;

    /* update item slot tab */
    memset(ItemSearch.item.name, '\0', STORAGE_NAME_LEN);
    memcpy(ItemSearch.item.name, STORAGE_FREEITEM_NAME, strlen(STORAGE_FREEITEM_NAME));
    ItemSearch.item.len = 0;
    ItemSearch.item.data_addr = 0;

    Storage_Comput_ItemSlot_CRC(&ItemSearch.item);
    if (Storage_ItemSlot_Update(ItemSearch.item_addr, ItemSearch.item_index, p_Sec, ItemSearch.item) != Storage_Error_None)
    {
        STORAGE_INFO("delete item", "failed");
        return Storage_ItemUpdate_Error;
    }

    return Storage_Error_None;
}

static Storage_ErrorCode_List Storage_CreateItem(Storage_ParaClassType_List _class, const char *name, uint8_t *p_data, uint16_t size)
{
    uint8_t *crc_buf = NULL;
    uint8_t *slot_update_ptr = NULL;
    uint16_t base_info_crc = 0;
    uint32_t storage_data_size = 0;
    uint32_t stored_size = 0;
    uint32_t store_addr = 0;
    uint32_t unstored_size = 0;
    uint32_t storage_tab_addr = 0;
    uint32_t cur_freeslot_addr = 0;
    uint32_t slot_useful_size = 0;
    uint8_t item_index = 0;
    Storage_FlashInfo_TypeDef *p_Flash = NULL;
    Storage_BaseSecInfo_TypeDef *p_Sec = NULL;
    Storage_Item_TypeDef *tab_item = NULL;
    Storage_Item_TypeDef crt_item_slot;
    Storage_FreeSlot_TypeDef FreeSlot;
    Storage_DataSlot_TypeDef DataSlot;
    Storage_ItemSearchOut_TypeDef search;
    uint8_t align_byte = 0;
    bool update_freeslot = true;

    memset(&search, 0, sizeof(Storage_ItemSearchOut_TypeDef));
    memset(&crt_item_slot, 0, sizeof(crt_item_slot));
    memset(&DataSlot, 0, sizeof(Storage_DataSlot_TypeDef));
    memset(&FreeSlot, 0, sizeof(Storage_FreeSlot_TypeDef));

    STORAGE_INFO("create", "name %s data size %d", name, size);

    /* can not name data as <Item_Avaliable> */
    if ((name == NULL) || (p_data == NULL) || (size == 0) || (memcmp(name, STORAGE_FREEITEM_NAME, strlen(STORAGE_FREEITEM_NAME)) == 0))
        return Storage_Param_Error;

    p_Flash = &Storage_Monitor.info;
    p_Sec = Storage_Get_SecInfo(p_Flash, _class);
    if (p_Sec == NULL)
        return Storage_Class_Error;

    if (p_Sec->free_slot_addr == 0)
        return Storage_FreeSlot_Addr_Error;

    /* search tab if matched item with the same name then abrot creating 
     * or give the free or empty storage item solt to create
     */
    if (p_Sec->para_num)
    {
        search = Storage_Search(_class, name);
        /* item name already exist */
        if (search.item_addr)
            return Storage_TabItem_Exist;
    }

    if (StorageDev.param_read(Storage_Monitor.ExtDev_ptr, p_Sec->free_slot_addr, (uint8_t *)&FreeSlot, sizeof(Storage_FreeSlot_TypeDef)) && \
        (FreeSlot.head_tag == STORAGE_SLOT_HEAD_TAG) && \
        (FreeSlot.end_tag == STORAGE_SLOT_END_TAG) && \
        (strlen(name) <= STORAGE_NAME_LEN))
    {
        cur_freeslot_addr = p_Sec->free_slot_addr;

        if (size % STORAGE_DATA_ALIGN)
        {
            align_byte = STORAGE_DATA_ALIGN - size % STORAGE_DATA_ALIGN;
        }
        else
            align_byte = 0;

        p_Sec->para_num ++;
        p_Sec->para_size += (size + align_byte);
        
        storage_tab_addr = p_Sec->tab_addr;
        for(uint16_t tab_i = 0; tab_i < p_Sec->tab_num; tab_i ++)
        {
            store_addr = 0;
            item_index = 0;

            /* step 1: update item slot */
            if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, storage_tab_addr, page_data_tmp, (p_Sec->tab_size / p_Sec->tab_num)))
                return Storage_Read_Error;

            tab_item = (Storage_Item_TypeDef *)page_data_tmp;
            for (uint16_t item_i = 0; item_i < Item_Capacity_Per_Tab; item_i ++)
            {
                /* current item slot is empty or is available (been free) */
                if (((tab_item[item_i].head_tag == STORAGE_ITEM_HEAD_TAG) && \
                     (tab_item[item_i].end_tag == STORAGE_ITEM_END_TAG) && \
                     (memcmp(tab_item[item_i].name, STORAGE_FREEITEM_NAME, strlen(STORAGE_FREEITEM_NAME)) == 0)) || \
                    ((tab_item[item_i].head_tag != STORAGE_ITEM_HEAD_TAG) && \
                     (tab_item[item_i].end_tag != STORAGE_ITEM_END_TAG)))
                {
                    item_index = item_i;

                    /* found empty item slot */
                    crt_item_slot = tab_item[item_i];

                    /* set item slot info */
                    crt_item_slot._class = _class;
                    memset(crt_item_slot.name, '\0', STORAGE_NAME_LEN);
                    memcpy(crt_item_slot.name, name, strlen(name));
                    crt_item_slot.len = size + align_byte;

                    /* set free slot address as current data address */
                    crt_item_slot.data_addr = p_Sec->free_slot_addr;
                    crt_item_slot.head_tag = STORAGE_ITEM_HEAD_TAG;
                    crt_item_slot.end_tag = STORAGE_ITEM_END_TAG;

                    /* comput crc */
                    Storage_Comput_ItemSlot_CRC(&crt_item_slot);
                    store_addr = crt_item_slot.data_addr;
                    break; 
                }
            }

            if (store_addr)
                break;

            storage_tab_addr += p_Sec->tab_size;
        }

        if (store_addr == 0)
            return Storage_DataAddr_Update_Error;

        storage_data_size = size;
        /* get align byte size */
        /* noticed: write 0 on align space */
        storage_data_size += align_byte;

        /* noticed: DataSlot.cur_data_size - DataSlot.align_size is current slot storaged data size */
        DataSlot.total_data_size = storage_data_size;
        unstored_size = DataSlot.total_data_size;

        if (p_Sec->free_space_size >= (storage_data_size + sizeof(Storage_DataSlot_TypeDef)))
        {
            while(true)
            {
                /* step 2: comput storage data size and set data slot */
                DataSlot.head_tag = STORAGE_SLOT_HEAD_TAG;
                DataSlot.end_tag = STORAGE_SLOT_END_TAG;
                
                STORAGE_INFO("create", "current free slot size %d", FreeSlot.size);

                if (FreeSlot.size <= sizeof(Storage_DataSlot_TypeDef))
                    return Storage_No_Enough_Space;

                p_data += stored_size;
                slot_useful_size = FreeSlot.size - sizeof(Storage_DataSlot_TypeDef);
                STORAGE_INFO("create", "free slot useful size %d store size %d", slot_useful_size, storage_data_size);
                /* current have space for new data need to be storage */
                if (slot_useful_size < storage_data_size)
                {
                    /*
                     * UNTESTED IN THIS BRANCH
                     */
                    
                    DataSlot.cur_slot_size = slot_useful_size;
                    stored_size += DataSlot.cur_slot_size;
                    unstored_size -= stored_size;
                    DataSlot.align_size = align_byte;
                    
                    /* current free slot full fill can not split any space for next free slot`s start */
                    DataSlot.nxt_addr = FreeSlot.nxt_addr;

                    /* in light of current free slot not enough for storage data, 
                     * then find next free slot used for storage data remaining */
                    cur_freeslot_addr = FreeSlot.nxt_addr;
                    if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, cur_freeslot_addr, (uint8_t *)&FreeSlot, sizeof(Storage_FreeSlot_TypeDef)))
                        return Storage_FreeSlot_Get_Error;   
                }
                else
                {
                    /* seperate data slot and new free slot from current free slot */
                    stored_size += unstored_size;
                    DataSlot.cur_slot_size = unstored_size;
                    DataSlot.align_size = align_byte;
                    DataSlot.nxt_addr = 0;

                    /* update current free slot adderess */
                    cur_freeslot_addr += DataSlot.cur_slot_size + sizeof(Storage_DataSlot_TypeDef);
                    FreeSlot.size -= DataSlot.cur_slot_size + sizeof(Storage_DataSlot_TypeDef);
                    
                    if (FreeSlot.size <= sizeof(Storage_DataSlot_TypeDef))
                    {
                        DataSlot.frag_len = FreeSlot.size;
                        FreeSlot.size = 0;

                        cur_freeslot_addr = FreeSlot.nxt_addr;
                        update_freeslot = false;
                    }
                }

                p_Sec->free_space_size -= DataSlot.cur_slot_size;

                /* write to the data section */
                /* storage target data */
                slot_update_ptr = page_data_tmp;
                memcpy(slot_update_ptr, &DataSlot.head_tag, sizeof(DataSlot.head_tag));
                slot_update_ptr += sizeof(DataSlot.head_tag);
                memcpy(slot_update_ptr, &DataSlot.total_data_size, sizeof(DataSlot.total_data_size));
                slot_update_ptr += sizeof(DataSlot.total_data_size);
                memcpy(slot_update_ptr, &DataSlot.cur_slot_size, sizeof(DataSlot.cur_slot_size));
                slot_update_ptr += sizeof(DataSlot.cur_slot_size);
                memcpy(slot_update_ptr, &DataSlot.nxt_addr, sizeof(DataSlot.nxt_addr));
                slot_update_ptr += sizeof(DataSlot.nxt_addr);
                memcpy(slot_update_ptr, &DataSlot.frag_len, sizeof(DataSlot.frag_len));
                slot_update_ptr += sizeof(DataSlot.frag_len);
                memcpy(slot_update_ptr, &DataSlot.align_size, sizeof(DataSlot.align_size));
                slot_update_ptr += sizeof(DataSlot.align_size);
                crc_buf = slot_update_ptr;
                memcpy(slot_update_ptr, p_data, (DataSlot.cur_slot_size - DataSlot.align_size));
                slot_update_ptr += (DataSlot.cur_slot_size - DataSlot.align_size);
                
                if (DataSlot.align_size)
                    memset(slot_update_ptr, 0, DataSlot.align_size);
                
                slot_update_ptr += DataSlot.align_size;

                /* comput current slot crc */
                DataSlot.slot_crc = Common_CRC16(crc_buf, DataSlot.cur_slot_size);
                memcpy(slot_update_ptr, &DataSlot.slot_crc, sizeof(DataSlot.slot_crc));
                slot_update_ptr += sizeof(DataSlot.slot_crc);
                memcpy(slot_update_ptr, &DataSlot.res, sizeof(DataSlot.res));
                slot_update_ptr += sizeof(DataSlot.res);
                memcpy(slot_update_ptr, &DataSlot.end_tag, sizeof(DataSlot.end_tag));
                slot_update_ptr += sizeof(DataSlot.end_tag);

                /* step 3: store data to data section */
                if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, store_addr, page_data_tmp, (DataSlot.cur_slot_size + sizeof(DataSlot))))
                    return Storage_Write_Error;

                if (DataSlot.nxt_addr == 0)
                {
                    if (DataSlot.total_data_size == stored_size)
                    {
                        /* step 4: update free slot */
                        if (update_freeslot && !StorageDev.param_write(Storage_Monitor.ExtDev_ptr, cur_freeslot_addr, (uint8_t *)&FreeSlot, sizeof(FreeSlot)))
                            return Storage_Write_Error;

                        break;
                    }

                    return Storage_No_Enough_Space;
                }
                else
                {
                    /* after target data segment stored, shift target data pointer to unstored pos
                     * and update next segment data store address */
                    stored_size += slot_useful_size;
                    store_addr = DataSlot.nxt_addr;
                }
            }
        }

        /* get tab */
        if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, storage_tab_addr, page_data_tmp, (p_Sec->tab_size / p_Sec->tab_num)))
            return Storage_Read_Error;

        tab_item = (Storage_Item_TypeDef *)page_data_tmp;
        tab_item[item_index] = crt_item_slot;

        /* write back item slot list to tab */
        if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, storage_tab_addr, page_data_tmp, (p_Sec->tab_size / p_Sec->tab_num)))
            return Storage_Write_Error;

        /* update free slot address in base info */
        p_Sec->free_slot_addr = cur_freeslot_addr;

        /* update base info crc */
        uint8_t *tmp_buf = &page_data_tmp[Storage_InfoPageSize];
        memset(tmp_buf, 0, Storage_InfoPageSize);
        memcpy(tmp_buf, p_Flash, sizeof(Storage_FlashInfo_TypeDef));
        base_info_crc = Common_CRC16(tmp_buf, Storage_InfoPageSize - sizeof(base_info_crc));
        memcpy(&tmp_buf[Storage_InfoPageSize - sizeof(base_info_crc)], &base_info_crc, sizeof(base_info_crc));
        
        /* update base info from section start*/
        if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, tmp_buf, Storage_InfoPageSize))
            return Storage_Write_Error;
        
        memset(tmp_buf, 0, Storage_InfoPageSize);
    }

    return Storage_Error_None;
}

static bool Storage_Establish_Tab(Storage_ParaClassType_List class)
{
    Storage_FlashInfo_TypeDef *p_Flash = &Storage_Monitor.info;
    Storage_BaseSecInfo_TypeDef *p_SecInfo = NULL;
    uint16_t clear_cnt = 0;
    uint32_t clear_byte = 0;
    uint32_t clear_remain = 0;
    uint32_t addr_tmp = 0;
    Storage_FreeSlot_TypeDef free_slot;

    switch ((uint8_t)class)
    {
        case Para_Sys:
            STORAGE_INFO("establish tab", "Building %s Tab", "Sys");
            p_SecInfo = &(p_Flash->sys_sec);
            break;
        
        case Para_User:
            STORAGE_INFO("establish tab", "Building %s Tab", "User");
            p_SecInfo = &(p_Flash->user_sec);
            break;

        default: STORAGE_INFO("establish tab", "Unknow type"); return false;
    }

    if (p_SecInfo->tab_addr && Storage_Clear_Tab(p_SecInfo->tab_addr, p_SecInfo->tab_num))
    {
        /* clear boot data section */
        if (p_SecInfo->data_sec_addr == 0)
            return false;

        /* write 0 to data section */
        memset(page_data_tmp, 0, Storage_TabSize);
        clear_cnt = p_SecInfo->data_sec_size / Storage_TabSize;
        clear_remain = p_SecInfo->data_sec_size;
        addr_tmp = p_SecInfo->data_sec_addr;
        clear_byte = Storage_TabSize;
        if (p_SecInfo->data_sec_size % Storage_TabSize)
            clear_cnt ++;
 
        for(uint16_t i = 0; i < clear_cnt; i++)
        {
            if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, addr_tmp, page_data_tmp, clear_byte))
                return false;

            addr_tmp += Storage_TabSize;
            clear_remain -= clear_byte;
            if (clear_remain && clear_remain <= clear_byte)
                clear_byte = clear_remain;
        }

        /* write free slot info */
        memset(&free_slot, 0, sizeof(free_slot));
        free_slot.head_tag = STORAGE_SLOT_HEAD_TAG;
        free_slot.size = p_SecInfo->data_sec_size - sizeof(free_slot);
        free_slot.nxt_addr = 0;
        /* if current free slot get enought space for data then set next address (nxt_addr) as 0 */
        /* or else setnext address (nxt_addr) as next slot address such as 0x80exxxx. */
        /* until all data was saved into multiple flash segment completely */
        /* set ender tag as 0xFE1001FE */
        free_slot.end_tag = STORAGE_SLOT_END_TAG;
        
        memcpy(page_data_tmp, &free_slot, sizeof(free_slot));
        if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, p_SecInfo->data_sec_addr, page_data_tmp, Storage_TabSize))
            return false;

        /* update info section */
        p_SecInfo->free_slot_addr = p_SecInfo->data_sec_addr;
        p_SecInfo->free_space_size = p_SecInfo->data_sec_size;
        p_SecInfo->para_num = 0;
        p_SecInfo->para_size = 0;

        /* read out whole section info data from storage info section */
        if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, page_data_tmp, Storage_TabSize))
            return false;

        if (!Storage_UpdateBaseInfo())
            return false;

        return true;
    }
    
    return false;
}

static bool Storage_Build_StorageInfo(void)
{
    Storage_FlashInfo_TypeDef Info_Rx;
    uint32_t tab_addr_offset = 0;
    uint32_t remain_data_sec_size = 0;
    uint32_t data_sec_size = 0;
    uint32_t base_addr = Storage_Monitor.info.base_addr;

    memset(&Info_Rx, 0, sizeof(Storage_FlashInfo_TypeDef));
    memset(&Storage_Monitor.info.sys_sec, 0, sizeof(Storage_BaseSecInfo_TypeDef));
    memset(&Storage_Monitor.info.user_sec, 0, sizeof(Storage_BaseSecInfo_TypeDef));
    memcpy(Storage_Monitor.info.tag, EXTERNAL_STORAGE_PAGE_TAG, EXTERNAL_PAGE_TAG_SIZE);
    memcpy(Storage_Monitor.info.version, version, STORAGE_VERSION_LEN);
    Storage_Monitor.info.total_size = Flash_Storage_TotalSize;

    /* fill reserve section */
    if (!Storage_Fill_ReserveSec(Storage_Monitor.info.base_addr + Storage_InfoPageSize))
        return false;

    /* set system data table section info
     * table address 
     * table size
     * table num
     * data section size
     * default parameter size
     * default parameter num
     */
    Storage_Monitor.info.sys_sec.tab_addr = base_addr + Storage_InfoPageSize + Storage_ReserveBlock_Size;
    Storage_Monitor.info.sys_sec.tab_num = TabNum;
    Storage_Monitor.info.sys_sec.tab_size = TabSize;
    Storage_Monitor.info.sys_sec.data_sec_size = Flash_SysDataSec_Size;
    Storage_Monitor.info.sys_sec.para_size = 0;
    Storage_Monitor.info.sys_sec.para_num = 0;
    tab_addr_offset = Storage_Monitor.info.sys_sec.tab_addr + TabSize + Storage_ReserveBlock_Size;
    
    /* fill 0x55 to reserve area */
    if (!Storage_Fill_ReserveSec(tab_addr_offset - Storage_ReserveBlock_Size))
        return false;

    /* set user data table section info
     * table address 
     * table size
     * table num
     * data section size
     * default parameter size
     * default parameter num
     */
    Storage_Monitor.info.user_sec.tab_addr = tab_addr_offset;
    Storage_Monitor.info.user_sec.tab_num = TabNum;
    Storage_Monitor.info.user_sec.tab_size = TabSize;
    Storage_Monitor.info.user_sec.data_sec_size = Flash_UserDataSec_Size;
    Storage_Monitor.info.user_sec.para_size = 0;
    Storage_Monitor.info.user_sec.para_num = 0;
    tab_addr_offset = Storage_Monitor.info.user_sec.tab_addr + TabSize + Storage_ReserveBlock_Size;

    /* fill 0x55 to reserve area */
    if (!Storage_Fill_ReserveSec(tab_addr_offset - Storage_ReserveBlock_Size))
        return false;
    
    STORAGE_INFO("build info", "total size 0x%08x", Storage_Monitor.info.total_size);
    STORAGE_INFO("build info", "tab addr offset 0x%08x", tab_addr_offset);
    /* get the remaining size of rom space has left */
    if (Storage_Monitor.info.total_size < tab_addr_offset)
    {
        STORAGE_INFO("build info", "table over range");
        Storage_Assert(true);
        return false;
    }

    /* remain_size = total size - (info_page_size - reserve_size) - (sys_table_size - reserve_size) - (user_table_size - reserve_size) */
    remain_data_sec_size = Storage_Monitor.info.total_size - (tab_addr_offset - base_addr);
    data_sec_size += Storage_Monitor.info.sys_sec.data_sec_size + Storage_ReserveBlock_Size;
    data_sec_size += Storage_Monitor.info.user_sec.data_sec_size + Storage_ReserveBlock_Size;

    STORAGE_INFO("build info", "Avilable size 0x%08x", remain_data_sec_size);
    STORAGE_INFO("build info", "Assigned size 0x%08x", data_sec_size);
    if (remain_data_sec_size < data_sec_size)
    {
        STORAGE_INFO("build info", "data section over range");
        Storage_Assert(true);
        return false;
    }

    Storage_Monitor.info.remain_size = remain_data_sec_size - data_sec_size;
    Storage_Monitor.info.data_sec_size = Storage_Monitor.info.sys_sec.data_sec_size + Storage_Monitor.info.user_sec.data_sec_size;

    /* get data sec addr */
    Storage_Monitor.info.sys_sec.data_sec_addr = tab_addr_offset;
    tab_addr_offset += Flash_SysDataSec_Size + Storage_ReserveBlock_Size;
    STORAGE_INFO("build info", "system data addr 0x%08x", Storage_Monitor.info.sys_sec.data_sec_addr);

    Storage_Monitor.info.user_sec.data_sec_addr = tab_addr_offset;
    tab_addr_offset += Flash_UserDataSec_Size + Storage_ReserveBlock_Size;
    STORAGE_INFO("build info", "user data addr 0x%08x", Storage_Monitor.info.user_sec.data_sec_addr);

    /* write 0 to info section */
    if (!Storage_UpdateBaseInfo())
        return false;

    /* read out again */
    if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, page_data_tmp, sizeof(Storage_FlashInfo_TypeDef)))
        return false;

    /* compare with target */
    memcpy(&Info_Rx, page_data_tmp, sizeof(Info_Rx));
    if (memcmp(&Info_Rx, &Storage_Monitor.info, sizeof(Storage_FlashInfo_TypeDef)) != 0)
        return false;

    if (!Storage_Establish_Tab(Para_Sys) || \
        !Storage_Establish_Tab(Para_User))
        return false;

    return true;
}

static bool Storage_Compare_ItemSlot_CRC(const Storage_Item_TypeDef item)
{
    uint8_t *crc_buf = (uint8_t *)&item;
    uint16_t crc_len = sizeof(Storage_Item_TypeDef);
    uint16_t crc = 0;
    
    crc_buf += sizeof(item.head_tag);
    crc_len -= sizeof(item.head_tag);
    crc_len -= sizeof(item.end_tag);
    crc_len -= sizeof(item.crc16);
    crc = Common_CRC16(crc_buf, crc_len);

    if (crc == item.crc16)
        return true;

    return false;
}

static bool Storage_Comput_ItemSlot_CRC(Storage_Item_TypeDef *p_item)
{
    uint8_t *crc_buf = NULL;
    uint16_t crc_len = sizeof(Storage_Item_TypeDef);
    
    if (p_item == NULL)
        return false;

    crc_buf = ((uint8_t *)p_item) + sizeof(p_item->head_tag);
    crc_len -= sizeof(p_item->head_tag);
    crc_len -= sizeof(p_item->end_tag);
    crc_len -= sizeof(p_item->crc16);
    p_item->crc16 = Common_CRC16(crc_buf, crc_len);

    return true;
}

static Storage_BaseSecInfo_TypeDef* Storage_Get_SecInfo(Storage_FlashInfo_TypeDef *info, Storage_ParaClassType_List class)
{
    if (info == NULL)
        return NULL;

    switch(class)
    {
        case Para_Sys:  return &(info->sys_sec);
        case Para_User: return &(info->user_sec);
        default:        return NULL;
    }
}

static bool Storage_Fill_ReserveSec(uint32_t addr)
{
    if (addr == 0)
    {
        STORAGE_INFO("set res area", "Invalid address");
        return false;
    }

    memset(page_data_tmp, Flash_Storage_ResData, Storage_ReserveBlock_Size);
    if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, addr, page_data_tmp, Storage_ReserveBlock_Size))
    {
        STORAGE_INFO("set res area", "write at 0x08%X failed", addr);
        Storage_Assert(true);
        return false;
    }

    memset(page_data_tmp, 0, Storage_ReserveBlock_Size);
    if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, addr, page_data_tmp, Storage_ReserveBlock_Size))
    {
        STORAGE_INFO("set res area", "read at 0x08%X failed", addr);
        Storage_Assert(true);
        return false;
    }

    for (uint16_t i = 0; i < Storage_ReserveBlock_Size; i ++)
    {
        if (page_data_tmp[i] != Flash_Storage_ResData)
        {
            STORAGE_INFO("set res area", "data at 0x08%X index at %d error data 0x02%X", addr, i, page_data_tmp[i]);
            Storage_Assert(true);
            return false;
        }
    }

    return true;
}

/************************************************************* Firmware API ****************************************************************/
static bool Storage_Erase_FirmwareSection(void)
{
    uint32_t Addr = Storage_Monitor.info.phy_start_addr + App_Firmware_PhyAddrOffset;
    return StorageDev.erase_phy_sec(Storage_Monitor.ExtDev_ptr, Addr, (App_Firmware_Size - 1));
}

static bool Storage_Read_Firmware(uint8_t *p_data, uint32_t size)
{
    StorageDevObj_TypeDef *p_DevObj = Storage_Monitor.ExtDev_ptr;
    static uint8_t *p_tmp = NULL;
    uint32_t Addr = Storage_Monitor.info.phy_start_addr + App_Firmware_PhyAddrOffset;
    uint32_t offset = 0;

    if ((p_data == NULL) || (size == 0) || (p_DevObj == NULL) || !Storage_Monitor.init_state)
        return false;

    if (p_tmp == NULL)
    {
        p_tmp = Storage_Malloc(p_DevObj->sector_size);
        if (p_tmp == NULL)
            return false;
    }

    while (size)
    {
        if (!StorageDev.read_phy_sec(Storage_Monitor.ExtDev_ptr, Addr, p_tmp, p_DevObj->sector_size))
            return false;

        if (size >= p_DevObj->sector_size)
        {
            memcpy(p_data + offset, p_tmp, p_DevObj->sector_size);
        }
        else
        {
            memcpy(p_data + offset, p_tmp, size);
            size = 0;
            return true;
        }

        size -= p_DevObj->sector_size;
        offset += p_DevObj->sector_size;
        Addr += p_DevObj->sector_size;
    }

    return false;
}

static bool Storage_Write_Firmware(uint8_t *p_data, uint32_t size)
{
    StorageDevObj_TypeDef *p_DevObj = Storage_Monitor.ExtDev_ptr;
    static uint8_t *p_tmp = NULL;
    uint32_t Addr = Storage_Monitor.info.phy_start_addr + App_Firmware_PhyAddrOffset;
    uint32_t offset = 0;

    if ((p_data == NULL) || (size == 0) || (p_DevObj == NULL) || !Storage_Monitor.init_state)
        return false;

    if (p_tmp == NULL)
    {
        p_tmp = Storage_Malloc(p_DevObj->sector_size);
        if (p_tmp == NULL)
            return false;
    }

    while (size)
    {
        if (size >= p_DevObj->sector_size)
        {
            memcpy(p_tmp, p_data + offset, p_DevObj->sector_size);
            size -= p_DevObj->sector_size;
        }
        else
        {
            if (!StorageDev.read_phy_sec(Storage_Monitor.ExtDev_ptr, Addr, p_tmp, p_DevObj->sector_size))
                return false;

            memcpy(p_tmp, p_data + offset, size);
            size = 0;
        }

        if (!StorageDev.write_phy_sec(Storage_Monitor.ExtDev_ptr, Addr, p_tmp, p_DevObj->sector_size))
            return false;

        offset += p_DevObj->sector_size;
        Addr += p_DevObj->sector_size;
    }

    return true;
}
