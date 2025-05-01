#ifndef __STORAGE_BUS_PORT_H
#define __STORAGE_BUS_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "HW_Def.h"

typedef void* (*StorageBus_Malloc_Callback)(uint32_t size);
typedef void (*StorageBus_Free_Callback)(void *ptr);

typedef struct
{
    void* (*init)(StorageBus_Malloc_Callback p_malloc, StorageBus_Free_Callback p_free);
#if (FLASH_CHIP_STATE == Storage_ChipBus_Spi)
    bool (*cs_ctl)(bool en);
    uint16_t (*bus_tx)(uint8_t *p_data, uint16_t len, uint32_t time_out);
    uint16_t (*bus_rx)(uint8_t *p_data, uint16_t len, uint32_t time_out);
    uint16_t (*bus_trans)(uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t time_out);
#elif (FLASH_CHIP_STATE == Storage_ChipBus_QSpi)
    bool (*bus_trans_cmd)();
    bool (*bus_status_polling)();
    bool (*bus_mem_map)();
    uint16_t (*bus_read)(uint8_t *p_rx);
    bool (*bus_write)(uint8_t *p_tx);
#endif
} StorageBusApi_TypeDef;

extern StorageBusApi_TypeDef StoragePort_Api;

#ifdef __cplusplus
}
#endif

#endif