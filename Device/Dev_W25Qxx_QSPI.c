#include "Dev_W25Qxx_QSPI.h"

#define DevQSPIW25Qxx_ChipErase_TIMEOUT_MAX     100000U     /* unit: s */
#define ErrToUint8(x)                           ((uint8_t) x)

/* internal function */
static bool DevQSPI_W25Qxx_ReadDevID(DevQSPIW25QxxObj_TypeDef *obj);
static bool DevQSPI_W25Qxx_AutoPollingMem(DevQSPIW25QxxObj_TypeDef *obj);
static bool DevQSPI_W25Qxx_WriteEnable(DevQSPIW25QxxObj_TypeDef *obj);

/* external function */
static uint8_t DevQSPI_W25Qxx_Init(DevQSPIW25QxxObj_TypeDef *obj);
static uint8_t DevQSPI_W25Qxx_Reset(DevQSPIW25QxxObj_TypeDef *obj);
static uint8_t DevQSPI_W25Qxx_MemMap(DevQSPIW25QxxObj_TypeDef *obj);
static uint8_t DevQSPI_W25Qxx_EraseChip(DevQSPIW25QxxObj_TypeDef *obj);
static uint8_t DevQSPI_W25Qxx_EraseAddr(DevQSPIW25QxxObj_TypeDef *obj, uint32_t addr);
static uint8_t DevQSPI_W25Qxx_Read(DevQSPIW25QxxObj_TypeDef *obj, uint32_t addr, uint8_t *p_data, uint16_t len);
static uint8_t DevQSPI_W25Qxx_WriteSector(DevQSPIW25QxxObj_TypeDef *obj, uint32_t addr, uint8_t *p_data, uint16_t len);
static DevNorFlash_Info_TypeDef DevQSPI_W25Qxx_GetInfo(DevQSPIW25QxxObj_TypeDef *obj);
static uint32_t DevW25Qxx_Get_Section_StartAddr(DevQSPIW25QxxObj_TypeDef *dev, uint32_t addr);

DevQSPIW25Qxx_TypeDef DevQSPIW25Qxx = {
    .Init = DevQSPI_W25Qxx_Init,
    .Reset = DevQSPI_W25Qxx_Reset,
    .MemoryMap = DevQSPI_W25Qxx_MemMap,
    .Erase_Chip = DevQSPI_W25Qxx_EraseChip,
    .Erase_Sector = DevQSPI_W25Qxx_EraseAddr,
    .Read_Sector = DevQSPI_W25Qxx_Read,
    .Write_Sector = DevQSPI_W25Qxx_WriteSector,
    .info = DevQSPI_W25Qxx_GetInfo,
    .get_section_start_addr = DevW25Qxx_Get_Section_StartAddr,
};

static uint8_t DevQSPI_W25Qxx_Init(DevQSPIW25QxxObj_TypeDef *obj)
{
    if (obj == NULL)
		return  ErrToUint8(QSPIW25Qxx_Obj_Error);

    obj->dev_id = 0;
    obj->init = false;

    /* read device ID */
    /* module reset */
    if (!DevQSPI_W25Qxx_ReadDevID(obj))
        return ErrToUint8(QSPIW25Qxx_ReadID_Failed);

    if (DevQSPI_W25Qxx_Reset(obj) != ErrToUint8(QSPIW25Qxx_Ok))
        return ErrToUint8(QSPIW25Qxx_Reset_Failed);

    obj->init = true;
    return ErrToUint8(QSPIW25Qxx_Ok);
}

static bool DevQSPI_W25Qxx_ReadDevID(DevQSPIW25QxxObj_TypeDef *obj)
{
	uint8_t	DeviceID_Tmp[3] = {0};
	
    if ((obj == NULL) || (obj->trans_cmd == NULL))
        return false;

    /* read deive id firset */
    if (!obj->trans_cmd(1, 0, 0, 0, 3, DevQSPIW25Qxx_CMD_JedecID) | !obj->read(DeviceID_Tmp))
        return false;

    obj->dev_id = (DeviceID_Tmp[0] << 16) | (DeviceID_Tmp[1] << 8 ) | DeviceID_Tmp[2];
    if (obj->dev_id != DevQSPIW25Qxx_FLASH_ID)
        return false;

    return true;
}

static uint8_t DevQSPI_W25Qxx_Reset(DevQSPIW25QxxObj_TypeDef *obj)
{
    if ((obj == NULL) || (obj->trans_cmd == NULL))
        return ErrToUint8(QSPIW25Qxx_Obj_Error);

    if (obj->busy)
        return ErrToUint8(QSPIW25Qxx_Busy);

    /* enable reset */
    obj->busy = true;
    if (!obj->trans_cmd(0, 0, 0, 0, 0, DevQSPIW25Qxx_CMD_EnableReset))
    {
        obj->busy = false;
        return ErrToUint8(QSPIW25Qxx_SendCMD_Error);
    }

    /* wait polling to the end */
    if (!DevQSPI_W25Qxx_AutoPollingMem(obj))
    {
        obj->busy = false;
        return ErrToUint8(QSPIW25Qxx_StatusPolling_Failed);
    }

    /* reset perform */
    if (!obj->trans_cmd(0, 0, 0, 0, 0, DevQSPIW25Qxx_CMD_ResetDevice))
    {
        obj->busy = false;
        return ErrToUint8(QSPIW25Qxx_SendCMD_Error);
    }
    
    /* wait polling to the end */
    if (!DevQSPI_W25Qxx_AutoPollingMem(obj))
    {
        obj->busy = false;
        return ErrToUint8(QSPIW25Qxx_StatusPolling_Failed);
    }

    obj->busy = false;
    return ErrToUint8(QSPIW25Qxx_Ok);
}

static bool DevQSPI_W25Qxx_AutoPollingMem(DevQSPIW25QxxObj_TypeDef *obj)
{
    if ((obj == NULL) || (obj->polling == NULL))
        return false;

    if (!obj->polling(1, 0, 0, 0, 0, DevQSPIW25Qxx_CMD_ReadStatus_REG1, 0, DevQSPIW25Qxx_Status_REG1_BUSY))
        return false;

    return true;
}

static uint8_t DevQSPI_W25Qxx_MemMap(DevQSPIW25QxxObj_TypeDef *obj)
{
    if ((obj == NULL) || (obj->mem_map == NULL))
        return ErrToUint8(QSPIW25Qxx_Obj_Error);

    /* reset */
    if (!DevQSPI_W25Qxx_Reset(obj))
        return ErrToUint8(QSPIW25Qxx_Reset_Failed);

    /* set memory mapped */
    if (!obj->mem_map(DevQSPIW25Qxx_CMD_FastReadQuad_IO))
        return ErrToUint8(QSPIW25Qxx_MemMap_Failed);

    return ErrToUint8(QSPIW25Qxx_Ok);
}

static bool DevQSPI_W25Qxx_WriteEnable(DevQSPIW25QxxObj_TypeDef *obj)
{
    if ((obj == NULL) || (obj->trans_cmd == NULL) || (obj->polling == NULL))
        return false;

    /* send enable command */
    if (!obj->trans_cmd(0, 0, 0, 0, 0, DevQSPIW25Qxx_CMD_WriteEnable))
        return false;
    
    /* polling state */
    if (!obj->polling(1, 0, 0, 0, 1, DevQSPIW25Qxx_CMD_ReadStatus_REG1, 0x02, DevQSPIW25Qxx_Status_REG1_WEL))
        return false;

    return true;
}

static uint8_t DevQSPI_W25Qxx_EraseAddr(DevQSPIW25QxxObj_TypeDef *obj, uint32_t addr)
{
    if ((obj == NULL) || (obj->trans_cmd == NULL))
        return ErrToUint8(QSPIW25Qxx_Obj_Error);

    if (obj->busy)
        return ErrToUint8(QSPIW25Qxx_Busy);

    obj->busy = true;
    if (!DevQSPI_W25Qxx_WriteEnable(obj))
    {
        obj->busy = false;
        return ErrToUint8(QSPIW25Qxx_WriteEnable_Failed);
    }

    /* send command */
    if (!obj->trans_cmd(0, 1, addr, 0, 0, DevQSPIW25Qxx_CMD_SectorErase))
    {
        obj->busy = false;
        return ErrToUint8(QSPIW25Qxx_SendCMD_Error);
    }

    /* polling status */
    if (!DevQSPI_W25Qxx_AutoPollingMem(obj))
    {
        obj->busy = false;
        return ErrToUint8(QSPIW25Qxx_StatusPolling_Failed);
    }

    obj->busy = false;
    return ErrToUint8(QSPIW25Qxx_Ok);
}

static uint8_t DevQSPI_W25Qxx_EraseChip(DevQSPIW25QxxObj_TypeDef *obj)
{
    if ((obj == NULL) || (obj->trans_cmd == NULL))
        return ErrToUint8(QSPIW25Qxx_Obj_Error);

    if (obj->busy)
        return ErrToUint8(QSPIW25Qxx_Busy);

    obj->busy = true;
    if (!DevQSPI_W25Qxx_WriteEnable(obj))
    {
        obj->busy = false;
        return ErrToUint8(QSPIW25Qxx_WriteEnable_Failed);
    }

    /* send clear command */
    if (!obj->trans_cmd(0, 0, 0, 0, 0, DevQSPIW25Qxx_CMD_ChipErase))
    {
        obj->busy = false;
        return ErrToUint8(QSPIW25Qxx_SendCMD_Error);
    }

    /* polling status */
    if (!obj->polling(1, 0, 0, 0, 0, DevQSPIW25Qxx_CMD_ReadStatus_REG1, 0x00, DevQSPIW25Qxx_Status_REG1_BUSY))
    {
        obj->busy = false;
        return ErrToUint8(QSPIW25Qxx_StatusPolling_Failed);
    }

    obj->busy = false;
    return ErrToUint8(QSPIW25Qxx_Ok);
}

/* read a sector */
static uint8_t DevQSPI_W25Qxx_WriteSector(DevQSPIW25QxxObj_TypeDef *obj, uint32_t addr, uint8_t *p_data, uint16_t len)
{
    uint32_t offset = 0;

    if ((obj == NULL) || (obj->trans_cmd == NULL) || \
        (obj->write == NULL) || (p_data == NULL) || (len == 0) || (addr % DevQSPIW25QXX_SectorSize != 0))
        return ErrToUint8(QSPIW25Qxx_Obj_Error);

    if (obj->busy)
        return ErrToUint8(QSPIW25Qxx_Busy);

    obj->busy = true;
    for (uint8_t i = 0; i < (DevQSPIW25QXX_SectorSize / DevQSPIW25Qxx_PageSize); i++)
    {
        /* set write enable */
        if (!DevQSPI_W25Qxx_WriteEnable(obj))
        {
            obj->busy = false;
            return ErrToUint8(QSPIW25Qxx_WriteEnable_Failed);
        }

        /* set write command */
        if (!obj->trans_cmd(4, 1, addr, 0, DevQSPIW25Qxx_PageSize, DevQSPIW25Qxx_CMD_QuadInputPageProgram))
        {
            obj->busy = false;
            return ErrToUint8(QSPIW25Qxx_SendCMD_Error);
        }

        /* transmit data */
        if (!obj->write(p_data + offset))
        {
            obj->busy = false;
            return ErrToUint8(QSPIW25Qxx_Write_Failed);
        }

        /* polling status */
        if (!DevQSPI_W25Qxx_AutoPollingMem(obj))
        {
            obj->busy = false;
            return ErrToUint8(QSPIW25Qxx_StatusPolling_Failed);
        }

        addr += DevQSPIW25Qxx_PageSize;
        offset += DevQSPIW25Qxx_PageSize;
    }

    obj->busy = false;
    return ErrToUint8(QSPIW25Qxx_Ok);
}

static uint8_t DevQSPI_W25Qxx_Read(DevQSPIW25QxxObj_TypeDef *obj, uint32_t addr, uint8_t *p_data, uint16_t len)
{
    if ((obj == NULL) || (obj->trans_cmd == NULL) || (obj->read == NULL) || (p_data == NULL) || (len == 0))
        return ErrToUint8(QSPIW25Qxx_Obj_Error);

    if (obj->busy)
        return ErrToUint8(QSPIW25Qxx_Busy);

    /* send read command */
    if (!obj->trans_cmd(4, 4, addr, 6, len, DevQSPIW25Qxx_CMD_FastReadQuad_IO))
    {
        obj->busy = false;
        return ErrToUint8(QSPIW25Qxx_SendCMD_Error);
    }

    /* receive data */
    if (!obj->read(p_data))
    {
        obj->busy = false;
        return ErrToUint8(QSPIW25Qxx_Read_Failed);
    }

    /* polling status */
    if (!DevQSPI_W25Qxx_AutoPollingMem(obj))
    {
        obj->busy = false;
        return ErrToUint8(QSPIW25Qxx_StatusPolling_Failed);
    }

    obj->busy = false;
    return ErrToUint8(QSPIW25Qxx_Ok);
}

static DevNorFlash_Info_TypeDef DevQSPI_W25Qxx_GetInfo(DevQSPIW25QxxObj_TypeDef *obj)
{
    DevNorFlash_Info_TypeDef info;

    memset(&info, 0, sizeof(DevNorFlash_Info_TypeDef));

    if ((obj != NULL) && obj->init)
    {
        info.start_addr = DevQSPIW25Qxx_Base_Address;
        info.page_size = DevQSPIW25Qxx_PageSize;
        info.block_size = DevQSPIW25Qxx_BlockSize;
        info.sector_size = DevQSPIW25QXX_SectorSize;
        info.prod_code = obj->dev_id;

        switch (obj->dev_id)
        {
            case (uint32_t)DevQSPIW25Qxx_FLASH_ID:
                info.flash_size = DevQSPIW25Qxx_FlashSize;
                info.page_num = DevQSPIW25Qxx_PageNum;
                info.block_num = DevQSPIW25Qxx_BlockNum;
                info.sector_num = DevQSPIW25Qxx_SectorNum;
                break;

            default: break;
        }
    }

    return info;
}

static uint32_t DevW25Qxx_Get_Section_StartAddr(DevQSPIW25QxxObj_TypeDef *dev, uint32_t addr)
{
    if (dev && dev->init)
        return (addr / DevQSPIW25QXX_SectorSize) * DevQSPIW25QXX_SectorSize;

    return 0;
}

