#include "Dev_W25Qxx_QSPI.h"

#define DevQSPIW25Qxx_ChipErase_TIMEOUT_MAX     100000U     /* unit: s */

static const DevQSPIW25Qxx_BusCMD_TypeDef Def_CMD = {
	.addr_line = AddrLine_1,
	.data_line = DataLine_4,
	.dummy_cycle = 0,
	.trans_size = 0,
	.cmd_code = 0,
    .addr = 0,
};

/* internal function */
static bool DevQSPI_W25Qxx_ReadDevID(DevQSPIW25QxxObj_TypeDef *obj);
static bool DevQSPI_W25Qxx_AutoPollingMem(DevQSPIW25QxxObj_TypeDef *obj);
static bool DevQSPI_W25Qxx_WriteEnable(DevQSPIW25QxxObj_TypeDef *obj);

/* external function */
static bool DevQSPI_W25Qxx_Init(DevQSPIW25QxxObj_TypeDef *obj);
static bool DevQSPI_W25Qxx_Reset(DevQSPIW25QxxObj_TypeDef *obj);
static bool DevQSPI_W25Qxx_EraseChip(DevQSPIW25QxxObj_TypeDef *obj);
static bool DevQSPI_W25Qxx_MemMap(DevQSPIW25QxxObj_TypeDef *obj);
static bool DevQSPI_W25Qxx_EraseAddr(DevQSPIW25QxxObj_TypeDef *obj, uint32_t addr);
static bool DevQSPI_W25Qxx_ReadPage(DevQSPIW25QxxObj_TypeDef *obj, uint32_t addr, uint8_t *p_data, uint16_t len);
static bool DevQSPI_W25Qxx_WritePage(DevQSPIW25QxxObj_TypeDef *obj, uint32_t addr, uint8_t *p_data, uint16_t len);

DevQSPIW25Qxx_TypeDef DevQSPIW25Qxx = {
    .Init = DevQSPI_W25Qxx_Init,
    .Reset = DevQSPI_W25Qxx_Reset,
    .MemoryMap = DevQSPI_W25Qxx_MemMap,
    .Erase_Chip = DevQSPI_W25Qxx_EraseChip,
    .Erase_Sector = DevQSPI_W25Qxx_EraseAddr,
    .Read_Sector = DevQSPI_W25Qxx_ReadPage,
    .Write_Sector = DevQSPI_W25Qxx_WritePage,
};

static bool DevQSPI_W25Qxx_Init(DevQSPIW25QxxObj_TypeDef *obj)
{
    if (obj == NULL)
		return false;

    obj->dev_id = 0;
    obj->init = false;

    /* read device ID */
    /* module reset */
    if (!DevQSPI_W25Qxx_ReadDevID(obj) | \
        !DevQSPI_W25Qxx_Reset(obj))
        return false;

    obj->init = true;
    return true;
}

static bool DevQSPI_W25Qxx_ReadDevID(DevQSPIW25QxxObj_TypeDef *obj)
{
	uint8_t	DeviceID_Tmp[3] = {0};    
	DevQSPIW25Qxx_BusCMD_TypeDef cmd_cfg = Def_CMD;
	
    if ((obj == NULL) || (obj->trans_cmd == NULL))
        return false;

    /* read deive id firset */
    cmd_cfg.addr_line  = AddrLine_None;
    cmd_cfg.data_line  = DataLine_1;
    cmd_cfg.trans_size = 3;
    cmd_cfg.cmd_code   = DevQSPIW25Qxx_CMD_JedecID;

    if (!obj->trans_cmd(cmd_cfg) | !obj->read(DeviceID_Tmp))
        return false;

    obj->dev_id = (DeviceID_Tmp[0] << 16) | (DeviceID_Tmp[1] << 8 ) | DeviceID_Tmp[2];
    if (obj->dev_id != DevQSPIW25Qxx_FLASH_ID)
        return false;

    return true;
}

static bool DevQSPI_W25Qxx_Reset(DevQSPIW25QxxObj_TypeDef *obj)
{
	DevQSPIW25Qxx_BusCMD_TypeDef cmd_cfg = Def_CMD;

    cmd_cfg.addr_line  = AddrLine_None;
    cmd_cfg.data_line  = DataLine_None;
    cmd_cfg.trans_size = 0;
    cmd_cfg.cmd_code   = DevQSPIW25Qxx_CMD_EnableReset;

    if ((obj == NULL) || (obj->trans_cmd == NULL))
        return false;

    /* enable reset */
    if (!obj->trans_cmd(cmd_cfg))
        return false;

    /* wait polling to the end */
    if (!DevQSPI_W25Qxx_AutoPollingMem(obj))
        return false;

    /* reset perform */
    cmd_cfg.cmd_code   = DevQSPIW25Qxx_CMD_ResetDevice;

    if (!obj->trans_cmd(cmd_cfg))
        return false;

    /* wait polling to the end */
    if (!DevQSPI_W25Qxx_AutoPollingMem(obj))
        return false;
    
    return true;
}

static bool DevQSPI_W25Qxx_AutoPollingMem(DevQSPIW25QxxObj_TypeDef *obj)
{
    DevQSPIW25Qxx_BusCMD_TypeDef cmd = Def_CMD;
    DevQSPIW25Qxx_BusCFG_TypeDef cfg;

    if ((obj == NULL) || (obj->polling == NULL))
        return false;

    memset(&cfg, 0, sizeof(DevQSPIW25Qxx_BusCFG_TypeDef));

    cmd.addr_line = AddrLine_None;
    cmd.data_line = DataLine_1;
    cmd.cmd_code  = DevQSPIW25Qxx_CMD_ReadStatus_REG1;

    cfg.match = 0;
    cfg.mask  = DevQSPIW25Qxx_Status_REG1_BUSY;

    if (!obj->polling(cmd, cfg))
        return false;

    return true;
}

static bool DevQSPI_W25Qxx_MemMap(DevQSPIW25QxxObj_TypeDef *obj)
{
    if ((obj == NULL) || (obj->mem_map == NULL))
        return false;

    /* reset */
    if (!DevQSPI_W25Qxx_Reset(obj))
        return false;

    /* set memory mapped */
    if (!obj->mem_map(DevQSPIW25Qxx_CMD_FastReadQuad_IO))
        return false;

    return true;
}

static bool DevQSPI_W25Qxx_WriteEnable(DevQSPIW25QxxObj_TypeDef *obj)
{
    DevQSPIW25Qxx_BusCMD_TypeDef cmd = Def_CMD;
    DevQSPIW25Qxx_BusCFG_TypeDef cfg;
    
    if ((obj == NULL) || (obj->trans_cmd == NULL) || (obj->polling == NULL))
        return false;

    cmd.addr_line = AddrLine_None;
    cmd.data_line = DataLine_None;
    cmd.cmd_code  = DevQSPIW25Qxx_CMD_WriteEnable;
        
    /* send enable command */
    if (!obj->trans_cmd(cmd))
        return false;
    
    cmd.data_line = DataLine_1;
    cmd.trans_size = 1;
    cmd.cmd_code = DevQSPIW25Qxx_CMD_ReadStatus_REG1;

    memset(&cfg, 0, sizeof(DevQSPIW25Qxx_BusCFG_TypeDef));
    cfg.mask = DevQSPIW25Qxx_Status_REG1_WEL;
    cfg.match = 0x02;

    /* polling state */
    if (!obj->polling(cmd, cfg))
        return false;

    return true;
}

static bool DevQSPI_W25Qxx_EraseAddr(DevQSPIW25QxxObj_TypeDef *obj, uint32_t addr)
{
    DevQSPIW25Qxx_BusCMD_TypeDef cmd = Def_CMD;

    if ((obj == NULL) || (obj->trans_cmd == NULL))
        return false;

    if (!DevQSPI_W25Qxx_WriteEnable(obj))
        return false;

    cmd.addr_line = AddrLine_1;
    cmd.data_line = DataLine_None;
    cmd.cmd_code  = DevQSPIW25Qxx_CMD_SectorErase;
    cmd.addr      = addr;

    /* send command */
    if (!obj->trans_cmd(cmd))
        return false;

    /* polling status */
    if (!DevQSPI_W25Qxx_AutoPollingMem(obj))
        return false;

    return true;
}

static bool DevQSPI_W25Qxx_EraseChip(DevQSPIW25QxxObj_TypeDef *obj)
{
    DevQSPIW25Qxx_BusCMD_TypeDef cmd = Def_CMD;
    DevQSPIW25Qxx_BusCFG_TypeDef cfg;
    
    if ((obj == NULL) || (obj->trans_cmd == NULL))
        return false;

    if (!DevQSPI_W25Qxx_WriteEnable(obj))
        return false;

    cmd.addr_line = AddrLine_None;
    cmd.data_line = DataLine_None;
    cmd.cmd_code  = DevQSPIW25Qxx_CMD_ChipErase;

    /* send clear command */
    if (!obj->trans_cmd(cmd))
        return false;

    cmd.data_line = DataLine_1;
    cmd.trans_size = 1;
    cmd.cmd_code = DevQSPIW25Qxx_CMD_ReadStatus_REG1;

    cfg.mask = DevQSPIW25Qxx_Status_REG1_BUSY;
    cfg.match = 0x00;

    /* polling status */
    if (!obj->polling(cmd, cfg))
        return false;

    return true;
}

static bool DevQSPI_W25Qxx_WritePage(DevQSPIW25QxxObj_TypeDef *obj, uint32_t addr, uint8_t *p_data, uint16_t len)
{
    DevQSPIW25Qxx_BusCMD_TypeDef cmd = Def_CMD;

    if ((obj == NULL) || (obj->trans_cmd == NULL) || \
        (obj->write == NULL) || (p_data == NULL) || (len == 0))
        return false;

    /* set write enable */
    if (!DevQSPI_W25Qxx_WriteEnable(obj))
        return false;

    cmd.addr_line  = AddrLine_1;
    cmd.data_line  = AddrLine_4;
    cmd.addr       = addr;
    cmd.trans_size = len;
    cmd.cmd_code   = DevQSPIW25Qxx_CMD_QuadInputPageProgram;

    /* set write command */
    if (!obj->trans_cmd(cmd))
        return false;

    /* transmit data */
    if (!obj->write(p_data))
        return false;

    /* polling status */
    if (!DevQSPI_W25Qxx_AutoPollingMem(obj))
        return false;

    return true;
}

static bool DevQSPI_W25Qxx_ReadPage(DevQSPIW25QxxObj_TypeDef *obj, uint32_t addr, uint8_t *p_data, uint16_t len)
{
    DevQSPIW25Qxx_BusCMD_TypeDef cmd = Def_CMD;

    if ((obj == NULL) || (obj->trans_cmd == NULL) || (obj->read == NULL) || (p_data == NULL) || (len == 0))
        return false;

    cmd.addr_line   = AddrLine_4;
    cmd.data_line   = AddrLine_4;
    cmd.dummy_cycle = 6;
    cmd.trans_size  = len;
    cmd.addr        = addr;
    cmd.cmd_code    = DevQSPIW25Qxx_CMD_FastReadQuad_IO;

    /* send read command */
    if (!obj->trans_cmd(cmd))
        return false;

    /* receive data */
    if (!obj->read(p_data))
        return false;

    /* polling status */
    if (!DevQSPI_W25Qxx_AutoPollingMem(obj))
        return false;

    return true;
}
