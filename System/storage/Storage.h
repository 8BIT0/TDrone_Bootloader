#ifndef __STORAGE_H
#define __STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "Storage_Def.h"
#include "Storage_Dev_Port.h"
#include "util.h"

typedef uint32_t storage_handle;

#define StorageItem_Size        sizeof(Storage_Item_TypeDef)
#define StorageItem_Align_Unit  64

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
    Storage_TabItem_Exist,
    Storage_SlotHeader_Error,
    Storage_Flash_NotAvailable,
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
    Storage_Merge_Error,
    Storage_Delete_Error,
} Storage_ErrorCode_List;

typedef enum
{
    Link_Done = 0,
    Link_Failed,
    Link_Searching,
} Link_State_List;

typedef enum
{
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

/* length must be 32Byte witchout payload data */
typedef struct
{
    uint32_t head_tag;
    uint16_t total_data_size;
    uint16_t cur_slot_size;
    uint32_t nxt_addr;
    uint16_t frag_len;
    uint16_t align_size;
    /* storage data insert */
    /*
     * for example: storage 13 byte name as "data_1" then data slot should be like the diagram down below
     *  _______________________________________________________________________________________________________________________________________________________________________
     * |    head    |   res   |  frag   | total data size | cur slot | nxt addr | align size | storage data |   align    |       slot crc    |     end    |   fregment area   |
     * | 0xEF0110EF |  data_1 |  len    |       16        |    16    |     0    |      3     | ............ |            |   comput crc by   | 0xFE1001FE |    (if have)      |
     * |   4Byte    |  8Byte  |  1Byte  |      4Byte      |  4Byte   |   4Byte  |    1Byte   |     13Byte   |   3Byte    | current slot data |     4Byte  |                   |
     * |____________|_________|_________|_________________|__________|__________|____________|______________|____________|___________________|____________|___________________|
     *                                                                                               |______________|               ↑
     *                                                                                                      |         16Byte        |
     *                                                                                                      |______ comput crc _____|
     * 
     */
    uint16_t slot_crc;
    uint16_t res;
    uint32_t end_tag;
} Storage_DataSlot_TypeDef;

typedef struct
{
    uint32_t head_tag;
    uint32_t size;
    uint32_t nxt_addr;
    uint32_t end_tag;
} Storage_FreeSlot_TypeDef;

typedef struct
{
    uint32_t tab_addr;
    uint32_t tab_num;
    uint32_t tab_size;
    uint32_t data_sec_addr;
    uint32_t data_sec_size;
    uint32_t free_slot_addr;
    uint32_t free_space_size;
    uint32_t para_size;
    uint32_t para_num;
} Storage_BaseSecInfo_TypeDef;

/* software storage info */
typedef struct
{
    uint8_t tag[32];
    uint8_t version[3];

    uint32_t phy_start_addr;
    uint32_t base_addr;

    uint32_t total_size;
    uint32_t remain_size;
    uint32_t data_sec_size;

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
    uint8_t Flash_Format_cnt;
    uint8_t Flash_ReInit_cnt;
    uint8_t Flash_BuildTab_cnt;
    uint8_t Flash_Error_Code;
    uint8_t Flash_Init_Error;   /* use for trace the place where the error occur */

    /* for some embed hardware devide */
    void *ExtDev_ptr;           /* external flash chip device obj pointer */
    void *ExtBusCfg_Ptr;        /* external flash chip hardware bus config data pointer */

    bool init_state;
    uint8_t inuse;

    uint32_t module_prod_code;

    Storage_FlashInfo_TypeDef info;
} Storage_Monitor_TypeDef;

typedef struct
{
    bool (*init)(StorageDevObj_TypeDef *ExtDev);
    Storage_ItemSearchOut_TypeDef (*search)(Storage_ParaClassType_List _class, const char *name);
    Storage_ErrorCode_List (*remove)(Storage_ParaClassType_List _class, const char *name);
    Storage_ErrorCode_List (*create)(Storage_ParaClassType_List _class, const char *name, uint8_t *p_data, uint16_t size);
    Storage_ErrorCode_List (*update)(Storage_ParaClassType_List _class, uint32_t data_addr , uint8_t *p_data, uint16_t size);
    Storage_ErrorCode_List (*get)(Storage_ParaClassType_List _class, Storage_Item_TypeDef item, uint8_t *p_data, uint16_t *size);
    Storage_ErrorCode_List (*get_dev_info)(StorageDevObj_TypeDef *info);
} Storage_TypeDef;

typedef struct
{
    bool (*erase)(void);
    bool (*read_sec)(uint8_t *p_data, uint32_t size);
    bool (*write_sec)(uint8_t *p_data, uint32_t size);
} StorageFirmware_TypeDef;

extern Storage_TypeDef Storage;
extern StorageFirmware_TypeDef StorageFirmware;

#ifdef __cplusplus
}
#endif

#endif
