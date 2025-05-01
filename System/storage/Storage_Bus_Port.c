#include "Storage_Bus_Port.h"

/* external function */
static void *Storage_External_Chip_Bus_Init(StorageBus_Malloc_Callback p_malloc, StorageBus_Free_Callback p_free);
#if (FLASH_CHIP_STATE == Storage_ChipBus_Spi)
static bool Storage_External_Chip_SelectPin_Ctl(bool state);
static uint16_t Storage_External_Chip_BusTx(uint8_t *p_data, uint16_t len, uint32_t time_out);
static uint16_t Storage_External_Chip_BusRx(uint8_t *p_data, uint16_t len, uint32_t time_out);
static uint16_t Storage_External_Chip_BusTrans(uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t time_out);
#elif (FLASH_CHIP_STATE == Storage_ChipBus_QSpi)

#endif

StorageBusApi_TypeDef StoragePort_Api = {
    .init               = Storage_External_Chip_Bus_Init,
#if (FLASH_CHIP_STATE == Storage_ChipBus_Spi)
    .bus_tx             = Storage_External_Chip_BusTx,
    .bus_rx             = Storage_External_Chip_BusRx,
    .bus_trans          = Storage_External_Chip_BusTrans,
    .cs_ctl             = Storage_External_Chip_SelectPin_Ctl,
#elif (FLASH_CHIP_STATE == Storage_ChipBus_QSpi)
    // .bus_trans_cmd      = ,
    // .bus_status_polling = ,
    // .bus_mem_map        = ,
    // .bus_read           = ,
    // .bus_write          = ,
#endif
};

/************************************************** External Flash IO API Section ************************************************/
static void* Storage_External_Chip_Bus_Init(StorageBus_Malloc_Callback p_malloc, StorageBus_Free_Callback p_free)
{
    void *obj = NULL;

    if ((p_malloc == NULL) || (p_free == NULL))
        return NULL;

#if (FLASH_CHIP_STATE == Storage_ChipBus_Spi)
    /* malloc bus object */
    obj = p_malloc(sizeof(BspSPI_Config_TypeDef));
    if (obj == NULL)
        return NULL;

    memset(obj, 0, sizeof(BspSPI_Config_TypeDef));

    To_NormalSPI_ObjPtr(obj)->BaudRatePrescaler = ExtFlash_Bus_Clock_Div;
    To_NormalSPI_ObjPtr(obj)->CLKPhase = ExtFlash_Bus_CLKPhase;
    To_NormalSPI_ObjPtr(obj)->CLKPolarity = ExtFlash_Bus_CLKPolarity;
    To_NormalSPI_ObjPtr(obj)->Instance = ExtFLash_Bus_Instance;
    To_NormalSPI_ObjPtr(obj)->Pin = ExtFlash_Bus_Pin;
    To_NormalSPI_ObjPtr(obj)->work_mode = BspSPI_Mode_Master;

    if (ExtFlash_Bus_Api.init(To_NormalSPI_Obj(obj), &ExtFlash_Bus_InstObj) && \
        BspGPIO.out_init(ExtFlash_CS_Pin))
        return obj;
#elif (FLASH_CHIP_STATE == Storage_ChipBus_QSpi)
    obj = p_malloc(sizeof(BspQSPI_Config_TypeDef));
    if (obj == NULL)
        return NULL;
    
    ExtFlash_Bus_InstObj.Instance = ExtFlash_Bus_Instance;
    ExtFlash_Bus_Api.init(obj, &ExtFlash_Bus_InstObj);
#endif
    
    return NULL;
}

#if (FLASH_CHIP_STATE == Storage_ChipBus_Spi)
static bool Storage_External_Chip_SelectPin_Ctl(bool state)
{
    BspGPIO.write(ExtFlash_CS_Pin, state);
    return true;
}

static uint16_t Storage_External_Chip_BusTx(uint8_t *p_data, uint16_t len, uint32_t time_out)
{
    if (p_data && len)
    {
        if (ExtFlash_Bus_Api.trans(&ExtFlash_Bus_InstObj, p_data, len, time_out))
            return len;
    }

    return 0;
}

static uint16_t Storage_External_Chip_BusRx(uint8_t *p_data, uint16_t len, uint32_t time_out)
{
    if (p_data && len)
    {
        if (ExtFlash_Bus_Api.receive(&ExtFlash_Bus_InstObj, p_data, len, time_out))
            return len;
    }

    return 0;
}

static uint16_t Storage_External_Chip_BusTrans(uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t time_out)
{
    if (tx && rx && len)
    {
        if (ExtFlash_Bus_Api.trans_receive(&ExtFlash_Bus_InstObj, tx, rx, len, time_out))
            return len;
    }

    return 0;
}
#elif (FLASH_CHIP_STATE == Storage_ChipBus_QSpi)

#endif
