#include "Storage_Bus_Port.h"

/* external function */
static void *Storage_External_Chip_Bus_Init(StorageBus_Malloc_Callback p_malloc, StorageBus_Free_Callback p_free);
#if (FLASH_CHIP_STATE == Storage_ChipBus_Spi)
static bool Storage_External_Chip_SelectPin_Ctl(bool state);
static uint16_t Storage_External_Chip_BusTx(uint8_t *p_data, uint16_t len, uint32_t time_out);
static uint16_t Storage_External_Chip_BusRx(uint8_t *p_data, uint16_t len, uint32_t time_out);
static uint16_t Storage_External_Chip_BusTrans(uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t time_out);
#elif (FLASH_CHIP_STATE == Storage_ChipBus_QSpi)
static bool Storage_External_Chip_Bus_TransCMD(uint8_t data_line, uint8_t addr_line, uint32_t addr, uint8_t dummy_cyc, uint16_t size, uint32_t cmd);
static bool Storage_External_Chip_Bus_Polling(uint8_t data_line, uint8_t addr_line, uint32_t addr, uint8_t dummy_cyc, uint16_t size, uint32_t cmd, uint32_t match, uint32_t mask);
static bool Storage_External_Chip_Bus_MemMap(uint32_t reg);
static bool Storage_External_Chip_Bus_Read(uint8_t *p_rx);
static bool Storage_External_Chip_Bus_Write(uint8_t *p_tx);
#endif

StorageBusApi_TypeDef StoragePort_Api = {
    .init               = Storage_External_Chip_Bus_Init,
#if (FLASH_CHIP_STATE == Storage_ChipBus_Spi)
    .bus_tx             = Storage_External_Chip_BusTx,
    .bus_rx             = Storage_External_Chip_BusRx,
    .bus_trans          = Storage_External_Chip_BusTrans,
    .cs_ctl             = Storage_External_Chip_SelectPin_Ctl,
#elif (FLASH_CHIP_STATE == Storage_ChipBus_QSpi)
    .bus_trans_cmd      = Storage_External_Chip_Bus_TransCMD,
    .bus_status_polling = Storage_External_Chip_Bus_Polling,
    .bus_mem_map        = Storage_External_Chip_Bus_MemMap,
    .bus_read           = Storage_External_Chip_Bus_Read,
    .bus_write          = Storage_External_Chip_Bus_Write,
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
    void *p_qspi = NULL;
    
    p_qspi = p_malloc(To_QSPI_Handle_Size);
    obj = p_malloc(sizeof(BspQSPI_Config_TypeDef));

    if ((obj == NULL) || (p_qspi == NULL))
    {
        if (obj)
            p_free(&obj);

        if (p_qspi)
            p_free(&p_qspi);

        return NULL;
    }

    To_QSPI_Handle_Ptr(p_qspi)->Instance = ExtFlash_Bus_Instance;
    if (!ExtFlash_Bus_Api.init(obj, p_qspi))
    {
        p_free(&obj);
        p_free(&p_qspi);
        return NULL;
    }

    ExtFlash_Bus_InstObj = (BspQSPI_Config_TypeDef *)obj;
#endif
    
    return obj;
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
static bool Storage_External_Chip_Bus_TransCMD(uint8_t data_line, uint8_t addr_line, uint32_t addr, uint8_t dummy_cyc, uint16_t size, uint32_t cmd)
{
    return ExtFlash_Bus_Api.cmd(ExtFlash_Bus_InstObj, data_line, addr_line, addr, dummy_cyc, size, cmd);
}

static bool Storage_External_Chip_Bus_Polling(uint8_t data_line, uint8_t addr_line, uint32_t addr, uint8_t dummy_cyc, uint16_t size, uint32_t cmd, uint32_t match, uint32_t mask)
{
    return ExtFlash_Bus_Api.polling(ExtFlash_Bus_InstObj, data_line, addr_line, addr, dummy_cyc, size, cmd, match, mask);
}

static bool Storage_External_Chip_Bus_MemMap(uint32_t reg)
{
    return ExtFlash_Bus_Api.memmap(ExtFlash_Bus_InstObj, reg);
}

static bool Storage_External_Chip_Bus_Read(uint8_t *p_rx)
{
    if (p_rx == NULL)
        return 0;

    return ExtFlash_Bus_Api.rx(ExtFlash_Bus_InstObj, p_rx);
}

static bool Storage_External_Chip_Bus_Write(uint8_t *p_tx)
{
    if (p_tx == NULL)
        return false;

    return ExtFlash_Bus_Api.tx(ExtFlash_Bus_InstObj, p_tx);
}

#endif
