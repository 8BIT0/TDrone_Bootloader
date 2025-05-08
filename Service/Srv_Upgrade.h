#ifndef __SRV_UPGRADE_H
#define __SRV_UPGRADE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

typedef void (*SrvUpgrade_Send_Callback)(void *port_obj, uint8_t *p_data, uint16_t len);

#pragma pack(1)
typedef struct
{
    bool update;
    bool compelet;
    uint32_t firmware_size;

    uint8_t res[128];
} SrvUpgrade_BaseInfo_TypeDef;
#pragma pack()

typedef enum
{
    Upgrade_Normal_Mode = 0,
    Upgrade_Force_Mode,
} SrvUpgrade_Mode_TypeDef;

typedef enum
{
    SrvUpgrade_Error_None = 0,
    SrvUpgrade_GetFirmareInfo_Error,
    SrvUpgrade_Receive_TimeOut,
    SrvUpgrade_PackData_Error,
} SrvUpgradeError_TypeDef;

typedef enum
{
    From_Ram = 0,   /* from ram */
    From_IRom,      /* from internal flash */
    From_ERom,      /* from external flash */
} SrvUpgrade_FirmwareDumpType_List;

typedef struct
{
    bool (*init)(SrvUpgrade_Send_Callback tx_cb);
    void (*DealRec)(void *com_obj, uint8_t *p_data, uint16_t size);
    bool (*DumpFirmware)(SrvUpgrade_FirmwareDumpType_List type, void *port, SrvUpgrade_Send_Callback tx_cb);
} SrvUpgrade_TypeDef;

extern SrvUpgrade_TypeDef SrvUpgrade;

#ifdef __cplusplus
}
#endif

#endif
