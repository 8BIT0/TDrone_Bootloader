#ifndef __DEV_W25QXX_QSPI_H
#define __DEV_W25QXX_QSPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>
#include "Dev_NorFlash_Def.h"

#define To_DevQSPIW25Qxx_API(x)                 ((DevQSPIW25Qxx_TypeDef *)x)
#define To_DevQSPIW25Qxx_OBJ(x)                 ((DevQSPIW25QxxObj_TypeDef *)x)

#define DevQSPIW25Qxx_CMD_EnableReset           0x66        /* reset enable */
#define DevQSPIW25Qxx_CMD_ResetDevice           0x99        /* device reset */
#define DevQSPIW25Qxx_CMD_JedecID               0x9F        /* device ID W25Q64 QSPI */
#define DevQSPIW25Qxx_CMD_WriteEnable           0X06        /* write enable */

#define DevQSPIW25Qxx_CMD_SectorErase           0x20        /* sector erase     4KByte  ref over head: 45ms */
#define DevQSPIW25Qxx_CMD_ChipErase             0xC7        /* whole chip erase 8MByte  ref over head: 20S */

#define DevQSPIW25Qxx_CMD_QuadInputPageProgram  0x32        /* 1-1-4 mode (1 wire command / 1 wire address / 4 wire data), page program command, program ref over head: 0.4ms */
#define DevQSPIW25Qxx_CMD_FastReadQuad_IO       0xEB        /* 1-4-4 mode (1 wire command / 4 wire address / 4 wire data), fast read command */

#define DevQSPIW25Qxx_CMD_ReadStatus_REG1       0X05        /* read status register 1 */
#define DevQSPIW25Qxx_Status_REG1_BUSY          0x01        /* read status register 1 bit0 (read only), Busy flag, bit set when erasing / write data / write command */
#define DevQSPIW25Qxx_Status_REG1_WEL           0x02        /* read status register 1 bit0 (read only), 'WEL' write enable flag, bit set indicate device writeable */

#define DevQSPIW25Qxx_PageNum                   32768
#define DevQSPIW25Qxx_PageSize                  256         /* page size        256Byte */
#define DevQSPIW25Qxx_SectorNum                 2048
#define DevQSPIW25QXX_SectorSize                0x1000      /* sector size      4KByte */
#define DevQSPIW25Qxx_FlashSize                 0x800000    /* W25Q64 ROM size  8MByte */
#define DevQSPIW25Qxx_BlockNum                  128
#define DevQSPIW25Qxx_BlockSize                 0x10000
#define DevQSPIW25Qxx_FLASH_ID                  0XEF4017    /* W25Q64 JEDEC ID */
#define DevQSPIW25Qxx_Mem_Addr                  0x90000000  /* memory map mode address */

#define DevQSPIW25Qxx_Base_Address              0x000000

typedef bool (*bus_cmd)(uint8_t data_line, uint8_t addr_line, uint32_t addr, uint8_t dummy_c, uint16_t size, uint32_t cmd);
typedef bool (*bus_polling)(uint8_t data_line, uint8_t addr_line, uint32_t addr, uint8_t dummy_c, uint16_t size, uint32_t cmd, uint32_t match, uint32_t mask);
typedef bool (*bus_mem_map)(uint32_t reg);
typedef bool (*bus_read)(uint8_t *p_rx);
typedef bool (*bus_write)(uint8_t *p_tx);

typedef enum
{
    QSPIW25Qxx_Ok = 0,
    QSPIW25Qxx_Busy,
    QSPIW25Qxx_Obj_Error,
    QSPIW25Qxx_ReadID_Failed,
    QSPIW25Qxx_Reset_Failed,
    QSPIW25Qxx_SendCMD_Error,
    QSPIW25Qxx_StatusPolling_Failed,
    QSPIW25Qxx_MemMap_Failed,
    QSPIW25Qxx_WriteEnable_Failed,
    QSPIW25Qxx_Write_Failed,
    QSPIW25Qxx_Read_Failed,
} DevQSPIW25Qxx_ErrorCode_TypeDef;

typedef struct
{
    bool init;
    uint32_t dev_id;

    bool busy;

    bus_cmd trans_cmd;
    bus_read read;
    bus_write write;
    bus_polling polling;
    bus_mem_map mem_map;
} DevQSPIW25QxxObj_TypeDef;

typedef struct
{
    uint8_t (*Init)(DevQSPIW25QxxObj_TypeDef *obj);
    uint8_t (*Reset)(DevQSPIW25QxxObj_TypeDef *obj);
    uint8_t (*MemoryMap)(DevQSPIW25QxxObj_TypeDef *obj);
    uint8_t (*Erase_Chip)(DevQSPIW25QxxObj_TypeDef *obj);
    uint8_t (*Erase_Sector)(DevQSPIW25QxxObj_TypeDef *obj, uint32_t addr);
    uint8_t (*Read_Sector)(DevQSPIW25QxxObj_TypeDef *obj, uint32_t addr, uint8_t *p_rx, uint16_t len);
    uint8_t (*Write_Sector)(DevQSPIW25QxxObj_TypeDef *obj, uint32_t addr, uint8_t *p_tx, uint16_t len);
    DevNorFlash_Info_TypeDef (*info)(DevQSPIW25QxxObj_TypeDef *dev);
    uint32_t (*get_section_start_addr)(DevQSPIW25QxxObj_TypeDef *dev, uint32_t addr);
} DevQSPIW25Qxx_TypeDef;

extern DevQSPIW25Qxx_TypeDef DevQSPIW25Qxx;

#ifdef __cplusplus
}
#endif
#endif
