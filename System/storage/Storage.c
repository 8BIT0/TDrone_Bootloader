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
#include "shell_port.h"
#include "util.h"
#include "HW_Def.h"
#include "Storage_Bus_Port.h"
#include "Storage_Dev_Port.h"
#include "Srv_OsCommon.h"
#include "debug_util.h"

#define STORAGE_TAG                     "[ STORAGE INFO ] "
#define STORAGE_INFO(fmt, ...)          Debug_Print(&DebugPort, STORAGE_TAG, fmt, ##__VA_ARGS__)

#define Item_Capacity_Per_Tab           (Storage_TabSize / sizeof(Storage_Item_TypeDef))

/* internal vriable */
Storage_Monitor_TypeDef Storage_Monitor;
#if (FLASH_CHIP_STATE == ON)
static uint8_t page_data_tmp[Storage_TabSize * 2] __attribute__((aligned(4))) = {0};
#else
static uint8_t page_data_tmp[1];
#endif

static bool Storage_Clear_Tab(uint32_t addr, uint32_t tab_num);
static bool Storage_Establish_Tab(Storage_ParaClassType_List class);

/* internal function */
static bool Storage_Build_StorageInfo(void);
static bool Storage_Get_StorageInfo(void);
static bool Storage_Format(void);
static bool Storage_Compare_ItemSlot_CRC(const Storage_Item_TypeDef item);
static bool Storage_Comput_ItemSlot_CRC(Storage_Item_TypeDef *p_item);
static Storage_BaseSecInfo_TypeDef* Storage_Get_SecInfo(Storage_FlashInfo_TypeDef *info, Storage_ParaClassType_List class);
static bool Storage_DeleteSingleDataSlot(uint32_t slot_addr, uint8_t *p_data, Storage_BaseSecInfo_TypeDef *p_Sec);
static Storage_ErrorCode_List Storage_FreeSlot_CheckMerge(uint32_t slot_addr, Storage_FreeSlot_TypeDef *slot_info, Storage_BaseSecInfo_TypeDef *p_Sec);
static bool Storage_Link_FreeSlot(uint32_t front_free_addr, uint32_t behind_free_addr, uint32_t new_free_addr, Storage_FreeSlot_TypeDef *new_free_slot);
static Storage_ErrorCode_List Storage_ItemSlot_Update(uint32_t tab_addr, uint8_t item_index, Storage_BaseSecInfo_TypeDef *p_Sec, Storage_Item_TypeDef item);
static void Storage_Module_Format(void);

/* external function */
static bool Storage_Init(StorageDevObj_TypeDef *ExtDev);
static Storage_ItemSearchOut_TypeDef Storage_Search(Storage_ParaClassType_List _class, const char *name);
static Storage_ErrorCode_List Storage_DeleteItem(Storage_ParaClassType_List _class, const char *name, uint32_t size);
static Storage_ErrorCode_List Storage_CreateItem(Storage_ParaClassType_List _class, const char *name, uint8_t *p_data, uint16_t size);
static Storage_ErrorCode_List Storage_SlotData_Update(Storage_ParaClassType_List _class, storage_handle data_slot_hdl, uint8_t *p_data, uint16_t size);
static Storage_ErrorCode_List Storage_Get_Data(Storage_ParaClassType_List _class, Storage_Item_TypeDef item, uint8_t *p_data, uint16_t size);
static Storage_ErrorCode_List Storage_Get_DevInfo(StorageDevObj_TypeDef *info);
static bool Storage_Write_Section(uint32_t addr, uint8_t *p_data, uint16_t len);
static bool Storage_Read_Section(uint32_t addr, uint8_t *p_data, uint16_t len);
static bool Storage_Erase_Section(uint32_t addr, uint16_t len);

static bool Storage_Firmware_Format(void);
static bool Storage_Frimware_Read(uint32_t addr_offset, uint8_t *p_data, uint16_t size);
static bool Storage_Firmware_Write(Storage_MediumType_List medium, uint32_t addr_offset, uint8_t *p_data, uint16_t size);

Storage_TypeDef Storage = {
    .init = Storage_Init,
    .search = Storage_Search,
    .create = Storage_CreateItem,
    .get = Storage_Get_Data,
    .update = Storage_SlotData_Update,
    .get_dev_info = Storage_Get_DevInfo,

    .write_section = Storage_Write_Section,
    .read_section = Storage_Read_Section,
    .erase_section = Storage_Erase_Section,

    .format_firmware = Storage_Firmware_Format,
    .read_firmware = Storage_Frimware_Read,
    .write_firmware = Storage_Firmware_Write,
};

static bool Storage_Init(StorageDevObj_TypeDef *ExtDev)
{
    void *bus_cfg = NULL;
    memset(&Storage_Monitor, 0, sizeof(Storage_Monitor));

    Storage_Monitor.init_state = false;
    if (!BspFlash.init())
        return false;

    /* external flash init */
#if (FLASH_CHIP_STATE == ON)
    if (ExtDev == NULL)
        return false;

    Storage_Monitor.ExtDev_ptr = NULL;
    bus_cfg = StoragePort_Api.init(SrvOsCommon.malloc, SrvOsCommon.free);
    if (bus_cfg == NULL)
    {
        STORAGE_INFO("Bus Init Failed\r\n");
        Storage_Monitor.ExternalFlash_Error_Code = Storage_BusInit_Error;
        return false;
    }

    STORAGE_INFO("Bus init accomplished\r\n");
    Storage_Monitor.ExtBusCfg_Ptr = bus_cfg;

    if (ExtDev->chip_type >= Storage_ChipType_All)
    {
        STORAGE_INFO("Unknown chip type\r\n");
        Storage_Monitor.ExternalFlash_Error_Code = Storage_ModuleType_Error;
        return false;
    }

    if (!StorageDev.set(ExtDev))
    {
        Storage_Monitor.ExternalFlash_Error_Code = Storage_ExtDevObj_Error;
        return false;
    }

    Storage_Monitor.ExtDev_ptr = ExtDev;
    Storage_Monitor.ExternalFlash_ReInit_cnt = ExternalModule_ReInit_Cnt;

reinit_external_flash_module:
    Storage_Monitor.ExternalFlash_Error_Code = Storage_Error_None;
    if (!StorageDev.init(ExtDev, &Storage_Monitor.module_prod_type, &Storage_Monitor.module_prod_code))
    {
        Storage_Monitor.ExternalFlash_Error_Code = Storage_ModuleInit_Error;
        STORAGE_INFO("chip init failed\r\n");
        if (Storage_Monitor.ExternalFlash_ReInit_cnt)
        {
            STORAGE_INFO("init retry remain %d\r\n\r\n", Storage_Monitor.ExternalFlash_ReInit_cnt);
            Storage_Monitor.ExternalFlash_ReInit_cnt --;
            goto reinit_external_flash_module;
        }
        return false;
    }

    /* set external flash device read write base address */
    Storage_Monitor.info.base_addr = ExtFlash_Start_Addr;
    Storage_Monitor.ExternalFlash_Format_cnt = Format_Retry_Cnt;
reupdate_external_flash_info:
    /* get storage info */
    if (!Storage_Get_StorageInfo())
    {
reformat_external_flash_info:
        if (Storage_Monitor.ExternalFlash_Format_cnt)
        {
            /* format storage device */
            if (!Storage_Format())
            {
                Storage_Monitor.ExternalFlash_Format_cnt --;
                Storage_Monitor.info.base_addr = ExtFlash_Start_Addr;
                if (Storage_Monitor.ExternalFlash_Format_cnt == 0)
                    return false;
                    
                goto reformat_external_flash_info;
            }

            /* external flash module format successed */
            /* build storage tab */
            if (Storage_Build_StorageInfo())
            {
                Storage_Monitor.ExternalFlash_BuildTab_cnt ++;
                Storage_Monitor.init_state = true;

                /* after tab builded read storage info again */
                goto reupdate_external_flash_info;
            }
        }
    }
    else
    {
        STORAGE_INFO("chip init done\r\n");
        Storage_Monitor.init_state = true;
    }
#endif

    return Storage_Monitor.init_state;
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
#if (FLASH_CHIP_ENABLE_STATE == ON)
    uint32_t size = 0;
    uint8_t default_data = 0;
    uint32_t read_time = 0;
    uint32_t remain_size = 0;
    uint32_t addr_offset = From_Start_Address;
        
    size = Storage_TabSize;
    default_data = ExtFlash_Storage_DefaultData;
    remain_size = ExtFlash_Storage_TotalSize;
    read_time = ExtFlash_Storage_TotalSize / Storage_TabSize;
    if (ExtFlash_Storage_TotalSize % Storage_TabSize)
        read_time ++;

    for(uint32_t i = 0; i < read_time; i++)
    {
        if ((remain_size != 0) && (remain_size < size))
            size = remain_size;

        if (!StorageDev.param_erase(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, addr_offset, size))
            return false;

        if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, addr_offset, page_data_tmp, size))
            return false;

        for(uint32_t j = 0; j < size; j++)
        {
            if (page_data_tmp[i] != default_data)
                return false;
        }

        addr_offset += size;
        remain_size -= size;

        if (remain_size == 0)
            return true;
    }
#endif
    return false;
}

static bool Storage_Check_Tab(Storage_BaseSecInfo_TypeDef *sec_info)
{
#if (FLASH_CHIP_ENABLE_STATE == ON)
    uint32_t free_i = 0;
    uint32_t tab_addr = 0;
    uint32_t store_param_found = 0;
    uint32_t store_param_size = 0;
    uint32_t free_slot_addr = 0;
    uint32_t sec_start_addr = 0;
    uint32_t sec_end_addr = 0;
    uint16_t crc16 = 0;
    uint8_t *crc_buf = NULL;
    uint16_t crc_len = 0;
    Storage_FreeSlot_TypeDef *FreeSlot_Info = NULL;
    Storage_Item_TypeDef *p_ItemList = NULL;

    if (sec_info == NULL)
        return false;

    /* free address & slot check */
    free_slot_addr = sec_info->free_slot_addr;
    sec_start_addr = sec_info->data_sec_addr;
    sec_end_addr = sec_start_addr + sec_info->data_sec_size;

    for(free_i = 0; ;)
    {
        /* check boot section free slot */
        if ((free_slot_addr == 0) || \
            (free_slot_addr < sec_start_addr) || \
            (free_slot_addr > sec_end_addr) || \
            !StorageDev.param_read(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, free_slot_addr, page_data_tmp, Storage_TabSize))
            return false;

        FreeSlot_Info = (Storage_FreeSlot_TypeDef *)page_data_tmp;

        if ((FreeSlot_Info->head_tag != STORAGE_SLOT_HEAD_TAG) || \
            (FreeSlot_Info->end_tag != STORAGE_SLOT_END_TAG))
            return false;

        free_i ++;
        if (FreeSlot_Info->nxt_addr == 0)
            break;

        free_slot_addr = FreeSlot_Info->nxt_addr;
    }
    
    if (sec_info->para_num)
    {
        tab_addr = sec_info->tab_addr;

        for (uint16_t tab_i = 0; tab_i < sec_info->page_num; tab_i ++)
        {
            if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, tab_addr, page_data_tmp, sec_info->tab_size / sec_info->page_num))
                return false;
        
            p_ItemList = (Storage_Item_TypeDef *)page_data_tmp;
            for(uint16_t item_i = 0; item_i < Item_Capacity_Per_Tab; item_i ++)
            {
                if ((p_ItemList[item_i].head_tag == STORAGE_ITEM_HEAD_TAG) && \
                    (p_ItemList[item_i].end_tag == STORAGE_ITEM_END_TAG))
                {
                    /* check item slot crc */
                    /*
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
                        * comput crc from class to len
                        */
                    crc_buf = (uint8_t *)&p_ItemList[item_i] + sizeof(p_ItemList[item_i].head_tag);
                    crc_len = sizeof(Storage_Item_TypeDef);
                    crc_len -= sizeof(p_ItemList[item_i].head_tag);
                    crc_len -= sizeof(p_ItemList[item_i].end_tag);
                    crc_len -= sizeof(p_ItemList[item_i].crc16);
                    store_param_size += p_ItemList[item_i].len;
                    
                    crc16 = Common_CRC16(crc_buf, crc_len);
                    if (crc16 != p_ItemList[item_i].crc16)
                        return false;

                    store_param_found ++;
                }
            }

            tab_addr += (sec_info->tab_size / sec_info->page_num);
        }

        if ((store_param_found != sec_info->para_num) || \
            (store_param_size != sec_info->para_size))
            return false;
    }

    return true;
#else
    return false;
#endif
}

static bool Storage_Get_StorageInfo(void)
{
#if (FLASH_CHIP_ENABLE_STATE == ON)
    Storage_FlashInfo_TypeDef *p_Info = NULL;
    Storage_FlashInfo_TypeDef Info_r;
    uint16_t crc = 0;
    uint16_t crc_read = 0;
    char flash_tag[INTERNAL_PAGE_TAG_SIZE + EXTERNAL_PAGE_TAG_SIZE];

    memset(flash_tag, '\0', sizeof(flash_tag));
    memset(&Info_r, 0, sizeof(Storage_FlashInfo_TypeDef));

    memcpy(flash_tag, EXTERNAL_STORAGE_PAGE_TAG, EXTERNAL_PAGE_TAG_SIZE);
    p_Info = &Storage_Monitor.info;
    
    if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, From_Start_Address, page_data_tmp, Storage_TabSize))
        return false;

    /* check internal storage tag */
    Info_r = *(Storage_FlashInfo_TypeDef *)page_data_tmp;
    
    /* check storage tag */
    /* check boot / sys / user  start addr */
    if ((strcmp((const char *)Info_r.tag, flash_tag) != 0) || \
        (Info_r.boot_sec.tab_addr == 0) || \
        (Info_r.sys_sec.tab_addr == 0) || \
        (Info_r.user_sec.tab_addr == 0) || \
        (Info_r.boot_sec.tab_addr == Info_r.sys_sec.tab_addr) || \
        (Info_r.boot_sec.tab_addr == Info_r.user_sec.tab_addr) || \
        (Info_r.sys_sec.tab_addr == Info_r.user_sec.tab_addr))
        return false;

    /* get crc from storage baseinfo section check crc value */
    memcpy(&crc_read, &page_data_tmp[Storage_InfoPageSize - sizeof(uint16_t)], sizeof(uint16_t));
    crc = Common_CRC16(page_data_tmp, Storage_InfoPageSize - sizeof(crc));
    if (crc != crc_read)
        return false;

    memset(page_data_tmp, 0, Storage_TabSize);
    /* check  boot  section tab & free slot info & stored item */
    /* check system section tab & free slot info & stored item */
    /* check  user  section tab & free slot info & stored item */
    if (Storage_Check_Tab(&Info_r.boot_sec) && \
        Storage_Check_Tab(&Info_r.sys_sec) && \
        Storage_Check_Tab(&Info_r.user_sec))
    {
        memcpy(p_Info, &Info_r, sizeof(Storage_FlashInfo_TypeDef));
        return true;
    }
#endif

    return false;
}

static bool Storage_Clear_Tab(uint32_t addr, uint32_t tab_num)
{
#if (FLASH_CHIP_ENABLE_STATE == ON)
    uint32_t addr_tmp = 0;

    if ((addr == 0) || \
        (tab_num == 0))
        return false;

    memset(page_data_tmp, 0, Storage_TabSize);
    addr_tmp = addr;
    
    for(uint32_t i = 0; i < tab_num; i++)
    {
        if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, addr_tmp, page_data_tmp, Storage_TabSize))
            return false;

        addr_tmp += Storage_TabSize;
    }

    return true;
#else
    return false;
#endif
}

/* 
 * if matched return data slot address 
 * else return 0
 */
static Storage_ItemSearchOut_TypeDef Storage_Search(Storage_ParaClassType_List _class, const char *name)
{
    Storage_ItemSearchOut_TypeDef ItemSearch;
#if (FLASH_CHIP_ENABLE_STATE == ON)
    Storage_BaseSecInfo_TypeDef *p_Sec = NULL;
    Storage_Item_TypeDef *item_list = NULL;
    Storage_Item_TypeDef *p_item = NULL;
    uint32_t tab_addr = 0;

    memset(&ItemSearch, 0, sizeof(ItemSearch));

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
    for (uint8_t tab_i = 0; tab_i < p_Sec->page_num; tab_i ++)
    {
        if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, tab_addr, page_data_tmp, (p_Sec->tab_size / p_Sec->page_num)))
            return ItemSearch;
    
        /* tab item traverse */
        item_list = (Storage_Item_TypeDef *)page_data_tmp;
        for (uint16_t item_i = 0; item_i < ((p_Sec->tab_size / p_Sec->page_num) / sizeof(Storage_Item_TypeDef)); item_i ++)
        {
            p_item = &item_list[item_i];

            if ((p_item->head_tag == STORAGE_ITEM_HEAD_TAG) && \
                (p_item->end_tag == STORAGE_ITEM_END_TAG) && \
                (memcmp(p_item->name, name, strlen(name)) == 0))
            {
                if (Storage_Compare_ItemSlot_CRC(*p_item))
                {
                    ItemSearch.item_addr = tab_addr;
                    ItemSearch.item_index = item_i;
                    ItemSearch.item = *p_item;
                    return ItemSearch;
                }
            }
        }
    
        /* update tab address */
        tab_addr += (p_Sec->tab_size / p_Sec->page_num);
    }
#else
    memset(&ItemSearch, 0, sizeof(ItemSearch));
#endif

    return ItemSearch;
}

static Storage_ErrorCode_List Storage_ItemSlot_Update(uint32_t tab_addr, uint8_t item_index, Storage_BaseSecInfo_TypeDef *p_Sec, Storage_Item_TypeDef item)
{
#if (FLASH_CHIP_ENABLE_STATE == ON)
    Storage_Item_TypeDef *ItemList = NULL;

    if ((tab_addr == 0) || \
        (p_Sec == NULL) || \
        (item.head_tag != STORAGE_ITEM_HEAD_TAG) || \
        (item.end_tag != STORAGE_ITEM_END_TAG) || \
        (item.data_addr == 0) || \
        (item.data_addr > (p_Sec->data_sec_addr + p_Sec->data_sec_size)) || \
        (item.data_addr < p_Sec->data_sec_addr))
        return Storage_Param_Error;

    if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, tab_addr, &page_data_tmp[Storage_TabSize], Storage_TabSize))
        return Storage_Read_Error;

    ItemList = (Storage_Item_TypeDef *)&page_data_tmp[Storage_TabSize];
    ItemList[item_index] = item;

    if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, tab_addr, (uint8_t *)ItemList, Storage_TabSize))
        return Storage_Write_Error;

    return Storage_Error_None;
#else
    return Storage_Read_Error;
#endif
}

static Storage_ErrorCode_List Storage_Get_Data(Storage_ParaClassType_List class, Storage_Item_TypeDef item, uint8_t *p_data, uint16_t size)
{
#if (FLASH_CHIP_ENABLE_STATE == ON)
    uint32_t data_len = 0;
    uint32_t data_addr = 0;
    uint8_t *p_read_out = NULL;
    uint8_t *crc_buf = NULL;
    uint16_t crc_len = 0;
    uint16_t crc = 0;
    Storage_DataSlot_TypeDef DataSlot;
    uint8_t *p_data_start = NULL;

    memset(&DataSlot, 0, sizeof(Storage_DataSlot_TypeDef));
    if (item.data_addr && p_data && size)
    {
        data_len = item.len;
        data_addr = item.data_addr;

        while(data_len)
        {
            if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, data_addr, page_data_tmp, data_len + sizeof(Storage_DataSlot_TypeDef)))
                return Storage_GetData_Error;

            p_read_out = page_data_tmp;
            memcpy(&DataSlot.head_tag, p_read_out, sizeof(DataSlot.head_tag));
            p_read_out += sizeof(DataSlot.head_tag);
            if (DataSlot.head_tag != STORAGE_SLOT_HEAD_TAG)
                return Storage_GetData_Error;

            memcpy(DataSlot.name, p_read_out, STORAGE_NAME_LEN);
            p_read_out += STORAGE_NAME_LEN;

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

            memcpy(&DataSlot.end_tag, p_read_out, sizeof(DataSlot.end_tag));
            if (DataSlot.end_tag != STORAGE_SLOT_END_TAG)
                return Storage_GetData_Error;

            memcpy(p_data, p_data_start, DataSlot.cur_slot_size - DataSlot.align_size);
            data_len -= DataSlot.cur_slot_size;

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

        return Storage_Error_None;
    }
#endif
    return Storage_GetData_Error;
}

static Storage_ErrorCode_List Storage_SlotData_Update(Storage_ParaClassType_List _class, storage_handle data_slot_hdl, uint8_t *p_data, uint16_t size)
{
#if (FLASH_CHIP_ENABLE_STATE == ON)
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
    {
        align_byte = STORAGE_DATA_ALIGN - size % STORAGE_DATA_ALIGN;
    }
    else
        align_byte = 0;

    /* get data slot first */
    if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, read_addr, p_read_tmp, sizeof(Storage_DataSlot_TypeDef)))
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
        p_read_tmp = page_data_tmp;
        p_slotdata = (Storage_DataSlot_TypeDef *)p_read_tmp;

        /* get data from handle */
        if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, read_addr, p_read_tmp, read_size))
            return Storage_Read_Error;

        p_slotdata->head_tag = *((uint32_t *)p_read_tmp);
        p_read_tmp += sizeof(p_slotdata->head_tag);
        if (p_slotdata->head_tag != STORAGE_SLOT_HEAD_TAG)
            return Storage_DataInfo_Error;

        memcpy(p_slotdata->name, p_read_tmp, STORAGE_NAME_LEN);
        p_read_tmp += STORAGE_NAME_LEN;

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
        if (p_slotdata->nxt_addr)
        {
            if ((p_slotdata->nxt_addr < p_Sec->data_sec_addr) || \
                (p_slotdata->nxt_addr > p_Sec->data_sec_addr + p_Sec->data_sec_size))
                return Storage_DataInfo_Error;
        }

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

        memcpy(p_read_tmp, &crc, sizeof(crc));
        
        p_data += p_slotdata->cur_slot_size - p_slotdata->align_size;
        p_read_tmp += sizeof(p_slotdata->slot_crc);

        if (*(uint32_t *)p_read_tmp != STORAGE_SLOT_END_TAG)
            return Storage_DataInfo_Error;

        if ((p_slotdata->nxt_addr == 0) && \
            (size == valid_data_size))
        {
            /* update data to flash */
            if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, read_addr, page_data_tmp, p_slotdata->cur_slot_size + sizeof(Storage_DataSlot_TypeDef)))
                return Storage_Write_Error;

            /* update accomplish */
            return Storage_Error_None;
        }

        read_addr = p_slotdata->nxt_addr;
    }

    return Storage_Error_None;
#else
    return Storage_Write_Error;
#endif
}

static bool Storage_Link_FreeSlot(uint32_t front_free_addr, uint32_t behind_free_addr, uint32_t new_free_addr, Storage_FreeSlot_TypeDef *new_free_slot)
{
#if (FLASH_CHIP_ENABLE_STATE == ON)
    Storage_FreeSlot_TypeDef front_slot;
    Storage_FreeSlot_TypeDef behind_slot;

    if ((new_free_addr == 0) || \
        (front_free_addr == 0) || \
        (behind_free_addr == 0) || \
        (front_free_addr >= new_free_addr) || \
        (behind_free_addr <= new_free_addr) || \
        (new_free_slot == NULL))
        return false;

    memset(&front_slot, 0, sizeof(Storage_FreeSlot_TypeDef));
    memset(&behind_slot, 0, sizeof(Storage_FreeSlot_TypeDef));

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

    if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, front_free_addr, (uint8_t *)&front_slot, sizeof(Storage_FreeSlot_TypeDef)))
        return false;

    /* link free slot address */
    front_slot.nxt_addr = new_free_addr;
    new_free_slot->nxt_addr = behind_free_addr;

    if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, front_free_addr, (uint8_t *)&front_slot, sizeof(Storage_FreeSlot_TypeDef)))
        return false;

    if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, new_free_addr, (uint8_t *)new_free_slot, sizeof(Storage_FreeSlot_TypeDef)))
        return false;

    return true;
#else
    return false;
#endif
}

/* developping & untested */
static Storage_ErrorCode_List Storage_FreeSlot_CheckMerge(uint32_t slot_addr, Storage_FreeSlot_TypeDef *slot_info, Storage_BaseSecInfo_TypeDef *p_Sec)
{
#if (FLASH_CHIP_ENABLE_STATE == ON)
    Storage_FreeSlot_TypeDef FreeSlot_Info;
    uint32_t front_freeslot_addr = 0;
    uint32_t behind_freeslot_addr = 0;
    uint32_t ori_freespace_size = 0;

    if ((p_Sec == NULL) || \
        (slot_info == NULL) || \
        (slot_addr < p_Sec->data_sec_addr) || \
        (slot_addr > (p_Sec->data_sec_addr + p_Sec->data_sec_size)))
        return Storage_Param_Error;

    memset(&FreeSlot_Info, 0, sizeof(FreeSlot_Info));

    if ((slot_info->head_tag != STORAGE_SLOT_HEAD_TAG) || \
        (slot_info->end_tag != STORAGE_SLOT_END_TAG) || \
        (slot_info->cur_slot_size > p_Sec->free_space_size))
        return Storage_FreeSlot_Info_Error;

    ori_freespace_size = p_Sec->free_space_size;
    front_freeslot_addr = p_Sec->free_slot_addr;
    while (true)
    {
        /* traverse all free slot */
        if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, front_freeslot_addr, (uint8_t *)&FreeSlot_Info, sizeof(FreeSlot_Info)))
            return Storage_Read_Error;

        if ((FreeSlot_Info.head_tag != STORAGE_SLOT_HEAD_TAG) || \
            (FreeSlot_Info.end_tag != STORAGE_SLOT_END_TAG))
            return Storage_FreeSlot_Info_Error;

        behind_freeslot_addr = FreeSlot_Info.nxt_addr;
        p_Sec->free_space_size += slot_info->cur_slot_size;

        /* circumstance 1: new free slot in front of the old free slot */
        if (slot_addr + slot_info->cur_slot_size == front_freeslot_addr)
        {
            slot_info->nxt_addr = FreeSlot_Info.nxt_addr;
            slot_info->cur_slot_size += FreeSlot_Info.cur_slot_size + sizeof(Storage_FreeSlot_TypeDef);

            memset(&FreeSlot_Info, 0, sizeof(FreeSlot_Info));

            /* write to front freeslot address */
            if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, front_freeslot_addr, (uint8_t *)&FreeSlot_Info, sizeof(FreeSlot_Info)))
            {
                p_Sec->free_space_size = ori_freespace_size;
                return Storage_Write_Error;
            }

            /* write to current freeslot section */
            if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, slot_addr, (uint8_t *)slot_info, sizeof(Storage_FreeSlot_TypeDef)))
            {
                p_Sec->free_space_size = ori_freespace_size;
                return Storage_Write_Error;
            }

            p_Sec->free_slot_addr = slot_addr;
            p_Sec->free_space_size += sizeof(Storage_FreeSlot_TypeDef);
        }
        /* circumstance 2: new free slot is behind of the old free slot */
        else if (front_freeslot_addr + FreeSlot_Info.cur_slot_size == slot_addr)
        {
            /* merge behind free slot */
            FreeSlot_Info.cur_slot_size += slot_info->cur_slot_size + sizeof(Storage_FreeSlot_TypeDef);
            Storage_Assert(slot_info->nxt_addr < FreeSlot_Info.nxt_addr);
            FreeSlot_Info.nxt_addr = slot_info->nxt_addr;

            memset(slot_info, 0, sizeof(Storage_FreeSlot_TypeDef));

            /* write to new free slot */
            if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, slot_addr, (uint8_t *)slot_info, sizeof(Storage_FreeSlot_TypeDef)))
                return Storage_Write_Error;

            /* write to behind free slot */
            if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, front_freeslot_addr, (uint8_t *)&FreeSlot_Info, sizeof(Storage_FreeSlot_TypeDef)))
                return Storage_Write_Error;
        }
        /* circumstance 3: none free slot near by */
        else if (((front_freeslot_addr + FreeSlot_Info.cur_slot_size + sizeof(Storage_FreeSlot_TypeDef)) < slot_addr) && \
                 (behind_freeslot_addr > (slot_addr + slot_info->cur_slot_size + sizeof(Storage_FreeSlot_TypeDef))))
        {
            /* link free slot */
            if (Storage_Link_FreeSlot(front_freeslot_addr, behind_freeslot_addr, slot_addr, slot_info))
                return Storage_Error_None;

            return Storage_FreeSlot_Link_Error;
        }

        if (FreeSlot_Info.nxt_addr == 0)
            return Storage_Error_None;

        /* update front free slot address */
        front_freeslot_addr = behind_freeslot_addr;
    }
#endif
    return Storage_Delete_Error;
}

/* untested */
static bool Storage_DeleteSingleDataSlot(uint32_t slot_addr, uint8_t *p_data, Storage_BaseSecInfo_TypeDef *p_Sec)
{
#if (FLASH_CHIP_ENABLE_STATE == ON)
    uint32_t cur_slot_size = 0;
    uint32_t inc_free_space = sizeof(Storage_DataSlot_TypeDef);
    uint8_t *p_freeslot_start = NULL;
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

    /* clear current slot name */
    memset(p_data, 0, STORAGE_NAME_LEN);
    p_data += STORAGE_NAME_LEN;

    /* clear total data size */
    *((uint32_t *)p_data) = 0;
    p_data += sizeof(uint32_t);

    /* get current slot data size */
    cur_slot_size = *((uint32_t *)p_data);
    inc_free_space += cur_slot_size;
    
    /* clear current slot data size */
    *((uint32_t *)p_data) = 0;
    p_data += sizeof(uint32_t);

    /* clear next data slot address */
    *((uint32_t *)p_data) = 0;
    p_data += sizeof(uint32_t);

    /* clear align size */
    *((uint8_t *)p_data) = 0;
    p_data += sizeof(uint8_t);

    /* clear data */
    memset(p_data, 0, cur_slot_size);
    p_data += cur_slot_size;

    /* clear crc */
    *((uint16_t *)p_data) = 0;
    p_data += sizeof(uint16_t);

    /* clear end tag */
    *((uint32_t *)p_data) = 0;

    /* update freeslot data info */
    if (*(uint32_t *)p_freeslot_start == STORAGE_SLOT_HEAD_TAG)
    {
        p_freeslot_start += sizeof(uint32_t);

        /* update current free slot size */
        *(uint32_t *)p_freeslot_start = cur_slot_size;
        p_freeslot_start += sizeof(uint32_t);

        /* reset next freeslot addr
         * link free slot address
         *
         * should link free slot
         */
        *(uint32_t *)p_freeslot_start = 0;
        p_freeslot_start += sizeof(uint32_t);

        /* set end tag */
        *(uint32_t *)p_freeslot_start = STORAGE_SLOT_END_TAG;
    }
    else
        return false;

    /* update to data section */
    if (StorageDev.param_write(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, slot_addr, data_w, inc_free_space))
    {
        /* check free slot and merge */
        if (Storage_FreeSlot_CheckMerge(slot_addr, (Storage_FreeSlot_TypeDef *)p_freeslot_start, p_Sec) == Storage_Error_None)
            return true;
    }
#endif
    return false;
}

/* developping & untested */
static bool Storage_DeleteAllDataSlot(uint32_t addr, char *name, uint32_t total_size, Storage_BaseSecInfo_TypeDef *p_Sec)
{
#if (FLASH_CHIP_ENABLE_STATE == ON)
    Storage_DataSlot_TypeDef data_slot;
    uint8_t *p_read = page_data_tmp;
    uint8_t name_len = 0;

    if ((addr == 0) || \
        (name == NULL) || \
        (strlen(name) == 0) || \
        (total_size == 0) || \
        (p_Sec == NULL))
        return false;

    memset(&data_slot, 0, sizeof(data_slot));
    name_len = strlen(name);

    if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, addr, page_data_tmp, total_size))
        return false;

    data_slot.head_tag = *((uint32_t *)p_read);
    if (data_slot.head_tag != STORAGE_SLOT_HEAD_TAG)
        return false;

    p_read += sizeof(data_slot.head_tag);
    memcpy(data_slot.name, p_read, STORAGE_NAME_LEN);
    if (memcmp(data_slot.name, name, name_len) != 0)
        return false;

    p_read += STORAGE_NAME_LEN;
    data_slot.total_data_size = *((uint32_t *)p_read);
    if (data_slot.total_data_size != total_size)
        return false;

    p_read += sizeof(data_slot.total_data_size);
    data_slot.cur_slot_size = *((uint32_t *)p_read);
    if (data_slot.cur_slot_size > data_slot.total_data_size)
        return false;

    p_read += sizeof(data_slot.cur_slot_size);
    data_slot.nxt_addr = *((uint32_t *)p_read);
    if ((data_slot.nxt_addr < p_Sec->data_sec_addr) || \
        (data_slot.nxt_addr > (p_Sec->data_sec_addr + p_Sec->data_sec_size)))
        return false;
    
    p_read += sizeof(data_slot.nxt_addr);
    data_slot.align_size = *((uint8_t *)p_read);    
    if (data_slot.align_size >= STORAGE_DATA_ALIGN)
        return false;

    /* set current slot data as 0 */
    p_read += sizeof(data_slot.align_size);
    memset(p_read, 0, data_slot.cur_slot_size);
    p_read += data_slot.cur_slot_size;

    /* clear crc */
    *((uint16_t *)p_read) = 0;
    p_read += sizeof(data_slot.slot_crc);

    /* ender error */
    if (*((uint32_t *)p_read) != STORAGE_SLOT_END_TAG)
        return false;

    /* delete next slot data */
    if (data_slot.nxt_addr)
    {
        /* traverse slot address */
        if (!Storage_DeleteAllDataSlot(data_slot.nxt_addr, name, total_size, p_Sec))
            return false;
    }

    /* reset data slot as free slot */
    if (!Storage_DeleteSingleDataSlot(addr, page_data_tmp, p_Sec))
        return false;

    return true;
#else
    return false;
#endif
}

/* developping */
static Storage_ErrorCode_List Storage_DeleteItem(Storage_ParaClassType_List _class, const char *name, uint32_t size)
{
#if (FLASH_CHIP_ENABLE_STATE == ON)
    Storage_FlashInfo_TypeDef *p_Flash = NULL;
    Storage_BaseSecInfo_TypeDef *p_Sec = NULL;
    Storage_ItemSearchOut_TypeDef ItemSearch;
    
    memset(&ItemSearch, 0, sizeof(ItemSearch));

    if ( !Storage_Monitor.init_state || \
        (name == NULL) || \
        (strlen(name) == 0) || \
        (strlen(name) >= STORAGE_NAME_LEN) || \
        (size == 0))
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
        return Storage_Delete_Error;

    /* update item slot tab */
    memset(ItemSearch.item.name, '\0', STORAGE_NAME_LEN);
    memcpy(ItemSearch.item.name, STORAGE_FREEITEM_NAME, strlen(STORAGE_FREEITEM_NAME));

    if (!Storage_Comput_ItemSlot_CRC(&ItemSearch.item))
        return Storage_ItemUpdate_Error;

    if (!Storage_ItemSlot_Update(ItemSearch.item_addr, ItemSearch.item_index, p_Sec, ItemSearch.item))
        return Storage_ItemUpdate_Error;

    /* update base info */
#endif
    return Storage_Delete_Error;
}

static Storage_ErrorCode_List Storage_CreateItem(Storage_ParaClassType_List _class, const char *name, uint8_t *p_data, uint16_t size)
{
#if (FLASH_CHIP_ENABLE_STATE == ON)
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
    uint8_t align_byte = 0;

    memset(&crt_item_slot, 0, sizeof(crt_item_slot));
    memset(&DataSlot, 0, sizeof(Storage_DataSlot_TypeDef));
    memset(&FreeSlot, 0, sizeof(Storage_FreeSlot_TypeDef));

    if ((name == NULL) || (p_data == NULL) || (size == 0))
        return Storage_Param_Error;

    p_Flash = &Storage_Monitor.info;
    p_Sec = Storage_Get_SecInfo(p_Flash, _class);
    if (p_Sec == NULL)
        return Storage_Class_Error;

    if (p_Sec->free_slot_addr == 0)
        return Storage_FreeSlot_Addr_Error;

    if (StorageDev.param_read(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, p_Sec->free_slot_addr, (uint8_t *)&FreeSlot, sizeof(Storage_FreeSlot_TypeDef)) && \
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
        for(uint16_t tab_i = 0; tab_i < p_Sec->page_num; tab_i ++)
        {
            store_addr = 0;
            item_index = 0;

            /* step 1: search tab */
            if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, storage_tab_addr, page_data_tmp, (p_Sec->tab_size / p_Sec->page_num)))
                return Storage_Read_Error;

            tab_item = (Storage_Item_TypeDef *)page_data_tmp;
            for (uint16_t item_i = 0; item_i < Item_Capacity_Per_Tab; item_i ++)
            {
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

            storage_tab_addr += (p_Sec->tab_size / p_Sec->page_num);
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

        if (p_Sec->free_space_size >= storage_data_size)
        {
            while(true)
            {
                /* step 2: comput storage data size and set data slot */
                DataSlot.head_tag = STORAGE_SLOT_HEAD_TAG;
                DataSlot.end_tag = STORAGE_SLOT_END_TAG;
                memset(DataSlot.name, '\0', STORAGE_NAME_LEN);
                memcpy(DataSlot.name, name, strlen(name));
                
                if (FreeSlot.cur_slot_size <= sizeof(Storage_DataSlot_TypeDef))
                    return Storage_No_Enough_Space;

                p_data += stored_size;
                slot_useful_size = FreeSlot.cur_slot_size - sizeof(Storage_DataSlot_TypeDef);
                /* current have space for new data need to be storage */
                if (slot_useful_size < storage_data_size)
                {
                    /*
                     * UNTESTED IN THIS BRANCH
                     */
                    
                    DataSlot.cur_slot_size = slot_useful_size;
                    stored_size += DataSlot.cur_slot_size;
                    unstored_size -= stored_size;
                    DataSlot.align_size = 0;
                    
                    /* current free slot full fill can not split any space for next free slot`s start */
                    DataSlot.nxt_addr = FreeSlot.nxt_addr;

                    /* in light of current free slot not enough for storage data, 
                        * then find next free slot used for storage data remaining */
                    cur_freeslot_addr = FreeSlot.nxt_addr;
                    if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, cur_freeslot_addr, (uint8_t *)&FreeSlot, sizeof(Storage_FreeSlot_TypeDef)))
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
                    FreeSlot.cur_slot_size -= DataSlot.cur_slot_size + sizeof(Storage_DataSlot_TypeDef);
                }

                p_Sec->free_space_size -= DataSlot.cur_slot_size;

                /* write to the data section */
                /* storage target data */
                slot_update_ptr = page_data_tmp;
                memcpy(slot_update_ptr, &DataSlot.head_tag, sizeof(DataSlot.head_tag));
                slot_update_ptr += sizeof(DataSlot.head_tag);
                memcpy(slot_update_ptr, DataSlot.name, STORAGE_NAME_LEN);
                slot_update_ptr += STORAGE_NAME_LEN;
                memcpy(slot_update_ptr, &DataSlot.total_data_size, sizeof(DataSlot.total_data_size));
                slot_update_ptr += sizeof(DataSlot.total_data_size);
                memcpy(slot_update_ptr, &DataSlot.cur_slot_size, sizeof(DataSlot.cur_slot_size));
                slot_update_ptr += sizeof(DataSlot.cur_slot_size);
                memcpy(slot_update_ptr, &DataSlot.nxt_addr, sizeof(DataSlot.nxt_addr));
                slot_update_ptr += sizeof(DataSlot.nxt_addr);
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
                memcpy(slot_update_ptr, &DataSlot.end_tag, sizeof(DataSlot.end_tag));
                slot_update_ptr += sizeof(DataSlot.end_tag);

                /* step 3: store data to data section */
                if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, store_addr, page_data_tmp, (DataSlot.cur_slot_size + sizeof(DataSlot))))
                    return Storage_Write_Error;

                if (DataSlot.nxt_addr == 0)
                {
                    if (DataSlot.total_data_size == stored_size)
                    {
                        /* step 4: update free slot */
                        if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, cur_freeslot_addr, (uint8_t *)&FreeSlot, sizeof(FreeSlot)))
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
        if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, storage_tab_addr, page_data_tmp, (p_Sec->tab_size / p_Sec->page_num)))
            return Storage_Read_Error;

        tab_item = (Storage_Item_TypeDef *)page_data_tmp;
        tab_item[item_index] = crt_item_slot;

        /* write back item slot list to tab */
        if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, storage_tab_addr, page_data_tmp, (p_Sec->tab_size / p_Sec->page_num)))
            return Storage_Write_Error;

        /* update free slot address in base info */
        p_Sec->free_slot_addr = cur_freeslot_addr;

        /* update base info crc */
        memcpy(page_data_tmp, p_Flash, sizeof(Storage_FlashInfo_TypeDef));
        base_info_crc = Common_CRC16(page_data_tmp, Storage_InfoPageSize - sizeof(base_info_crc));
        memcpy(&page_data_tmp[Storage_InfoPageSize - sizeof(base_info_crc)], &base_info_crc, sizeof(base_info_crc));
        
        /* update base info from section start*/
        if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, From_Start_Address, page_data_tmp, Storage_InfoPageSize))
            return Storage_Write_Error;
    }

    return Storage_Error_None;
#else
    return Storage_No_Enough_Space;
#endif
}

static bool Storage_Establish_Tab(Storage_ParaClassType_List class)
{
#if (FLASH_CHIP_ENABLE_STATE == ON)
    Storage_FlashInfo_TypeDef *p_Flash = NULL;
    Storage_BaseSecInfo_TypeDef *p_SecInfo = NULL;
    uint16_t clear_cnt = 0;
    uint32_t clear_byte = 0;
    uint32_t clear_remain = 0;
    uint32_t addr_tmp = 0;
    Storage_FreeSlot_TypeDef free_slot;
    uint16_t crc = 0;

    p_Flash = &Storage_Monitor.info;
    p_SecInfo = Storage_Get_SecInfo(p_Flash, class);
    if (p_SecInfo == NULL)
        return false;

    if (p_SecInfo->tab_addr && Storage_Clear_Tab(p_SecInfo->tab_addr, p_SecInfo->page_num))
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
            if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, addr_tmp, page_data_tmp, clear_byte))
                return false;

            addr_tmp += Storage_TabSize;
            clear_remain -= clear_byte;
            if (clear_remain && clear_remain <= clear_byte)
                clear_byte = clear_remain;
        }

        /* write free slot info */
        memset(&free_slot, 0, sizeof(free_slot));
        free_slot.head_tag = STORAGE_SLOT_HEAD_TAG;
        free_slot.cur_slot_size = p_SecInfo->data_sec_size - sizeof(free_slot);
        free_slot.nxt_addr = 0;
        /* if current free slot get enought space for data then set next address (nxt_addr) as 0 */
        /* or else setnext address (nxt_addr) as next slot address such as 0x80exxxx. */
        /* until all data was saved into multiple flash segment completely */
        /* set ender tag as 0xFE1001FE */
        free_slot.end_tag = STORAGE_SLOT_END_TAG;
        
        memcpy(page_data_tmp, &free_slot, sizeof(free_slot));
        if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, p_SecInfo->data_sec_addr, page_data_tmp, Storage_TabSize))
            return false;

        /* update info section */
        p_SecInfo->free_slot_addr = p_SecInfo->data_sec_addr;
        p_SecInfo->free_space_size = p_SecInfo->data_sec_size;
        p_SecInfo->para_num = 0;
        p_SecInfo->para_size = 0;

        /* read out whole section info data from storage info section */
        if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, From_Start_Address, page_data_tmp, Storage_TabSize))
            return false;

        memset(page_data_tmp, 0, Storage_InfoPageSize);
        memcpy(page_data_tmp, p_Flash, sizeof(Storage_FlashInfo_TypeDef));

        /* comput crc */
        crc = Common_CRC16(page_data_tmp, Storage_InfoPageSize - sizeof(crc));
        memcpy(&page_data_tmp[Storage_InfoPageSize - sizeof(crc)], &crc, sizeof(crc));

        /* erase sector first then write into the target sector */
        if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, From_Start_Address, page_data_tmp, Storage_TabSize))
            return false;

        return true;
    }
#endif
    return false;
}

static bool Storage_Build_StorageInfo(void)
{
#if (FLASH_CHIP_ENABLE_STATE == ON)
    uint32_t page_num = 0;
    Storage_FlashInfo_TypeDef Info;
    Storage_FlashInfo_TypeDef Info_Rx;
    uint32_t addr_offset = 0;
    uint32_t BaseInfo_start_addr = 0;
    uint32_t tab_addr_offset = 0;
    uint16_t crc = 0;
    uint32_t remain_data_sec_size = 0;
    uint32_t data_sec_size = 0;

    memset(&Info, 0, sizeof(Storage_FlashInfo_TypeDef));
    memset(&Info_Rx, 0, sizeof(Storage_FlashInfo_TypeDef));
    memcpy(Info.tag, EXTERNAL_STORAGE_PAGE_TAG, EXTERNAL_PAGE_TAG_SIZE);

    Info.total_size = ExtFlash_Storage_TotalSize;
    Info.base_addr = Storage_Monitor.info.base_addr;
    
    BaseInfo_start_addr = Info.base_addr;
    page_num = Storage_ExtFlash_Max_Capacity / (ExtFlash_Storage_TabSize / StorageItem_Size);
    if (page_num == 0)
        return false;
    
    Info.boot_sec.tab_addr = Storage_InfoPageSize;
    Info.boot_sec.tab_size = BootSection_Block_Size * BootTab_Num;
    Info.boot_sec.page_num = BootTab_Num;
    Info.boot_sec.data_sec_size = ExternalFlash_BootDataSec_Size;
    Info.boot_sec.para_size = 0;
    Info.boot_sec.para_num = 0;
    tab_addr_offset = (Info.boot_sec.tab_addr + Info.boot_sec.tab_size) + Storage_ReserveBlock_Size;

    Info.sys_sec.tab_addr = tab_addr_offset;
    Info.sys_sec.tab_size = page_num * ExtFlash_Storage_TabSize;
    Info.sys_sec.data_sec_size = ExternalFlash_SysDataSec_Size;
    Info.sys_sec.page_num = page_num;
    Info.sys_sec.para_size = 0;
    Info.sys_sec.para_num = 0;
    tab_addr_offset += Info.sys_sec.tab_size + Storage_ReserveBlock_Size;
        
    Info.user_sec.tab_addr = tab_addr_offset;
    Info.user_sec.tab_size = page_num * ExtFlash_Storage_TabSize;
    Info.user_sec.data_sec_size = ExternalFlash_UserDataSec_Size;
    Info.user_sec.page_num = page_num;
    Info.user_sec.para_size = 0;
    Info.user_sec.para_num = 0;
    tab_addr_offset += Info.user_sec.tab_size + Storage_ReserveBlock_Size;
    
    /* get the remaining size of rom space has left */
    if (Info.total_size < tab_addr_offset)
        return false;
    
    remain_data_sec_size = Info.total_size - tab_addr_offset;
    data_sec_size += Info.boot_sec.data_sec_size + Storage_ReserveBlock_Size;
    data_sec_size += Info.sys_sec.data_sec_size + Storage_ReserveBlock_Size;
    data_sec_size += Info.user_sec.data_sec_size + Storage_ReserveBlock_Size;

    if (remain_data_sec_size < data_sec_size)
        return false;
        
    Info.remain_size = remain_data_sec_size - data_sec_size;
    Info.data_sec_size = Info.boot_sec.data_sec_size + Info.sys_sec.data_sec_size + Info.user_sec.data_sec_size;

    /* get data sec addr */
    Info.boot_sec.data_sec_addr = tab_addr_offset;
    tab_addr_offset += ExternalFlash_BootDataSec_Size;
    tab_addr_offset += Storage_ReserveBlock_Size;

    Info.sys_sec.data_sec_addr = tab_addr_offset;
    tab_addr_offset += ExternalFlash_SysDataSec_Size;
    tab_addr_offset += Storage_ReserveBlock_Size;

    Info.user_sec.data_sec_addr = tab_addr_offset;
    tab_addr_offset += ExternalFlash_UserDataSec_Size;
    tab_addr_offset += Storage_ReserveBlock_Size;

    Storage_Monitor.info = Info;
    addr_offset = BaseInfo_start_addr - Storage_Monitor.info.base_addr;

    /* write 0 to info section */
    memset(page_data_tmp, 0, Storage_InfoPageSize);

    /* read out and erase sector */
    if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, addr_offset, page_data_tmp, sizeof(Info)))
        return false;

    /* write base info to info section */
    memcpy(page_data_tmp, &Info, sizeof(Info));
    crc = Common_CRC16(page_data_tmp, Storage_InfoPageSize - sizeof(crc));
    memcpy(&page_data_tmp[Storage_InfoPageSize - sizeof(crc)], &crc, sizeof(crc));

    /* write into flash chip */
    if (!StorageDev.param_write(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, addr_offset, page_data_tmp, Storage_InfoPageSize))
        return false;

    /* read out again */
    if (!StorageDev.param_read(Storage_Monitor.ExtDev_ptr, Storage_Monitor.info.base_addr, addr_offset, page_data_tmp, sizeof(Info)))
        return false;

    /* compare with target */
    memcpy(&Info_Rx, page_data_tmp, sizeof(Info_Rx));
    if (memcmp(&Info_Rx, &Info, sizeof(Storage_FlashInfo_TypeDef)) != 0)
        return false;

    if (!Storage_Establish_Tab(Para_Boot) || \
        !Storage_Establish_Tab(Para_Sys)  || \
        !Storage_Establish_Tab(Para_User))
        return false;

    volatile uint8_t test;
    test ++;
    return true;
#else
    return false;
#endif
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
        case Para_Boot: return &(info->boot_sec);
        case Para_Sys:  return &(info->sys_sec);
        case Para_User: return &(info->user_sec);
        default:        return NULL;
    }
}

/********************************************** BlackBox Storage API Section *****************************************************/
static bool Storage_Write_Section(uint32_t addr, uint8_t *p_data, uint16_t len)
{
    void *p_dev = Storage_Monitor.ExtDev_ptr;

    if (p_dev == NULL)
        return false;

    return StorageDev.write_phy_sec(To_StorageDevObj_Ptr(p_dev), addr, p_data, len);
}

static bool Storage_Read_Section(uint32_t addr, uint8_t *p_data, uint16_t len)
{
    void *p_dev = Storage_Monitor.ExtDev_ptr;

    if (p_dev == NULL)
        return false;

    return StorageDev.read_phy_sec(To_StorageDevObj_Ptr(p_dev), addr, p_data, len);
}

static bool Storage_Erase_Section(uint32_t addr, uint16_t len)
{
    void *p_dev = Storage_Monitor.ExtDev_ptr;

    if (p_dev == NULL)
        return false;

    return StorageDev.erase_phy_sec(To_StorageDevObj_Ptr(p_dev), addr, len);
}

/********************************************** External Firmware Storage API Section ********************************************/
static bool Storage_Firmware_Format(void)
{
    StorageDevObj_TypeDef *dev = To_StorageDevObj_Ptr(Storage_Monitor.ExtDev_ptr);
    
    if ((dev == NULL) || !Storage_Monitor.init_state)
        return false;

    return StorageDev.firmware_format(dev, App_Firmware_Addr, App_Firmware_Size);
}

static bool Storage_Frimware_Read(uint32_t addr_offset, uint8_t *p_data, uint16_t size)
{
    StorageDevObj_TypeDef *dev = To_StorageDevObj_Ptr(Storage_Monitor.ExtDev_ptr);

    if ((dev == NULL) || \
        !Storage_Monitor.init_state || \
        (p_data == NULL) || \
        (size == 0))
        return false;
        
    return StorageDev.firmware_read(dev, App_Firmware_Addr, addr_offset, p_data, size);
}

static bool Storage_Firmware_Write(Storage_MediumType_List medium, uint32_t addr_offset, uint8_t *p_data, uint16_t size)
{
    uint32_t write_addr = 0;
    StorageDevObj_TypeDef *dev = NULL;

    if ((p_data == NULL) || (size == 0))
        return false;

    if (medium == Internal_Flash)
    {
        write_addr = App_Address_Base + addr_offset;
        return BspFlash.write(write_addr, p_data, size);
    }
    
    if (medium == External_Flash)
    {
        dev = To_StorageDevObj_Ptr(Storage_Monitor.ExtDev_ptr);
        if (dev == NULL)
            return false;

        return StorageDev.firmware_write(dev, App_Firmware_Addr, addr_offset, p_data, size);
    }

    return false;
}
