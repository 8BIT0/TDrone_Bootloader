#ifndef __STORAGE_H
#define __STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>
#include "Bsp_Flash.h"
#include "Bsp_GPIO.h"
#include "Dev_W25Qxx.h"
#include "Srv_OsCommon.h"
#include "Storage_Def.h"
#include "Storage_Bus_Port.h"
#include "Storage_Dev_Port.h"
#include "util.h"

typedef uint32_t storage_handle;

#define StorageItem_Size sizeof(Storage_Item_TypeDef)

typedef enum
{
    Storage_Error_None = 0,
    Storage_Param_Error,
    Storage_BusInit_Error,
    Storage_BusType_Error,
    Storage_ExtDevObj_Error,
    Storage_ModuleType_Error,
    Storage_ModuleInit_Error,
    Storage_ModuleAPI_Error,
    Storage_Read_Error,
    Storage_Write_Error,
    Storage_Erase_Error,
    Storage_NameMatched,
    Storage_SlotHeader_Error,
    Storage_ExternalFlash_NotAvailable,
    Storage_InternalFlash_NotAvailable,
    Storage_Class_Error,
    Storage_RW_Api_Error,
    Storage_No_Enough_Space,
    Storage_DataInfo_Error,
    Storage_DataSize_Overrange,
    Storage_BaseInfo_Updata_Error,
    Storage_DataAddr_Update_Error,
    Storage_FreeSlot_Update_Error,
    Storage_FreeSlot_Get_Error,
    Storage_FreeSlot_Addr_Error,
    Storage_FreeSlot_Info_Error,
    Storage_FreeSlot_Link_Error,
    Storage_ItemInfo_Error,
    Storage_ItemUpdate_Error,
    Storage_GetData_Error,
    Storage_CRC_Error,
    Storage_Update_DataSize_Error,
    Storage_Delete_Error,
} Storage_ErrorCode_List;

typedef enum
{
    Internal_Flash = 0,
    External_Flash,
} Storage_MediumType_List;

typedef enum
{
    Para_Boot = 0,
    Para_Sys,
    Para_User,
} Storage_ParaClassType_List;

#pragma pack(1)
/* length must be 64Byte */
typedef struct
{
    uint8_t head_tag;
    uint8_t _class;
    uint8_t name[STORAGE_NAME_LEN];
    uint32_t data_addr;
    uint16_t len;
    uint8_t reserved[12];
    uint16_t crc16;
    uint8_t end_tag;
} Storage_Item_TypeDef;

/* length must be 64Byte witchout payload data */
typedef struct
{
    uint32_t head_tag;
    uint8_t name[STORAGE_NAME_LEN];
    uint32_t total_data_size;
    uint32_t cur_slot_size;
    uint32_t nxt_addr;
    uint8_t align_size;
    /* storage data insert */
    /*
     * for example: storage 13 byte name as "data_1" then data slot should be like the diagram down below
     *  _________________________________________________________________________________________________________________________________________
     * |    head    |   Name  | total data size | cur slot | nxt addr | align size | storage data |   align    |       slot crc    |     end    |
     * | 0xEF0110EF |  data_1 |       16        |    16    |     0    |      3     | ............ |            |   comput crc by   | 0xFE1001FE |
     * |   4Byte    |  41Byte |      4Byte      |  4Byte   |   4Byte  |    1Byte   |     13Byte   |   3Byte    | current slot data |     4Byte  |
     * |____________|_________|_________________|__________|__________|____________|______________|____________|___________________|____________|
     *                                                                                     |______________|               ↑
     *                                                                                            |         16Byte        |
     *                                                                                            |______ comput crc _____|
     * 
     */
    uint16_t slot_crc;
    uint32_t end_tag;
} Storage_DataSlot_TypeDef;

typedef struct
{
    uint32_t head_tag;
    uint32_t cur_slot_size;
    uint32_t nxt_addr;
    uint32_t end_tag;
} Storage_FreeSlot_TypeDef;

typedef struct
{
    uint32_t tab_addr;
    uint32_t data_sec_addr;
    uint32_t data_sec_size;
    uint32_t page_num;
    uint32_t tab_size;
    uint32_t free_slot_addr;
    uint32_t free_space_size;
    uint32_t para_size;
    uint32_t para_num;
} Storage_BaseSecInfo_TypeDef;

/* software storage info */
typedef struct
{
    uint8_t tag[32];

    uint32_t base_addr;

    uint32_t total_size;
    uint32_t remain_size;
    uint32_t data_sec_size;

    Storage_BaseSecInfo_TypeDef boot_sec;
    Storage_BaseSecInfo_TypeDef sys_sec;
    Storage_BaseSecInfo_TypeDef user_sec;
} Storage_FlashInfo_TypeDef;

typedef struct
{
    uint32_t item_addr;
    uint8_t item_index;
    Storage_Item_TypeDef item;
} Storage_ItemSearchOut_TypeDef;
#pragma pack()

typedef struct
{
    uint8_t InternalFlash_Format_cnt;
    uint8_t ExternalFlash_Format_cnt;
    uint8_t ExternalFlash_ReInit_cnt;

    uint8_t InternalFlash_BuildTab_cnt;
    uint8_t ExternalFlash_BuildTab_cnt;
    
    uint8_t ExternalFlash_Error_Code;
    uint8_t ExternalFlash_Init_Error; /* use for trace the place where the error occur */

    void *ExtDev_ptr;       /* external flash chip device obj pointer */
    void *ExtBusCfg_Ptr;    /* external flash chip hardware bus config data pointer */

    bool init_state;
    uint8_t inuse;

    uint16_t module_prod_type;
    uint16_t module_prod_code;

    Storage_FlashInfo_TypeDef info;
} Storage_Monitor_TypeDef;

typedef struct
{
    bool (*init)(StorageDevObj_TypeDef *ExtDev);
    Storage_ItemSearchOut_TypeDef (*search)(Storage_ParaClassType_List _class, const char *name);
    Storage_ErrorCode_List (*create)(Storage_ParaClassType_List _class, const char *name, uint8_t *p_data, uint16_t size);
    Storage_ErrorCode_List (*update)(Storage_ParaClassType_List _class, uint32_t addr , uint8_t *p_data, uint16_t size);
    Storage_ErrorCode_List (*get)(Storage_ParaClassType_List _class, Storage_Item_TypeDef item, uint8_t *p_data, uint16_t size);
    Storage_ErrorCode_List (*get_dev_info)(StorageDevObj_TypeDef *info);

    /* blackbox section */
    bool (*write_section)(uint32_t addr, uint8_t *p_data, uint16_t len);
    bool (*read_section)(uint32_t addr, uint8_t *p_data, uint16_t len);
    bool (*erase_section)(uint32_t addr, uint16_t len);

    /* firmware section */
    bool (*format_firmware)(void);
    bool (*read_firmware)(uint32_t addr_offset, uint8_t *p_date, uint16_t size);
    bool (*write_firmware)(Storage_MediumType_List medium, uint32_t addr_offset, uint8_t *p_data, uint16_t size);
} Storage_TypeDef;

extern Storage_TypeDef Storage;

#ifdef __cplusplus
}
#endif

#endif
