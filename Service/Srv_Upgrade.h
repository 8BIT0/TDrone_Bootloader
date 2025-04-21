#ifndef __SRV_UPGRADE_H
#define __SRV_UPGRADE_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef void (*SrvUpgrade_Send_Callback)(void *port_obj, uint8_t *p_data, uint16_t len);

#pragma pack(1)
typedef struct
{
    bool update;
    uint8_t sw_ver[3];
    uint32_t firmware_size;
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

typedef struct
{
    bool (*init)(SrvUpgrade_Send_Callback tx_cb);
    void (*DealRec)(void *com_obj, uint8_t *p_data, uint16_t size);
} SrvUpgrade_TypeDef;

extern SrvUpgrade_TypeDef SrvUpgrade;

#endif
