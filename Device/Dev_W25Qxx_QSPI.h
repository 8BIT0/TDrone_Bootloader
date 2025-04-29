#ifndef __DEV_W25QXX_QSPI_H
#define __DEV_W25QXX_QSPI_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>

typedef uint16_t (*DevQSPIW25Qxx_Bus_Trans_Callback)(uint32_t addr, uint8_t *p_tx, uint8_t *p_rx, uint16_t len);
typedef uint16_t (*DevQSPIW25Qxx_Bus_SendCMD_Callback)(uint32_t addr, void *cmd, uint16_t len);

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

#define DevQSPIW25Qxx_PageSize                  256         /* page size        256Byte */
#define DevQSPIW25Qxx_FlashSize                 0x800000    /* W25Q64 ROM size  8MByte */
#define DevQSPIW25Qxx_FLASH_ID                  0XEF4017    /* W25Q64 JEDEC ID */
#define DevQSPIW25Qxx_Mem_Addr                  0x90000000  /* memory map mode address */

typedef enum
{
    AddrLine_None = 0,
    AddrLine_1,
    AddrLine_4,
} DevQSPIW25Qxx_AddrLine_TypeDef;

typedef enum
{
    DataLine_None = 0,
    DataLine_1,
    DataLine_4,
} DevQSPIW25Qxx_DataLine_TypeDef;

typedef struct
{
    DevQSPIW25Qxx_AddrLine_TypeDef addr_Line;
    DevQSPIW25Qxx_DataLine_TypeDef data_line;
    uint32_t dummy_cycle;
    uint32_t cmd_code;
} DevQSPIW25Qxx_CMD_TypeDef;

typedef bool (*bus_cmd)(void *bus_obj, DevQSPIW25Qxx_CMD_TypeDef cmd);
typedef uint16_t (*bus_read)(void *bus_obj, uint8_t *p_rx, uint16_t size);

typedef struct
{
    bool init;
    uint32_t dev_id;

    bus_cmd trans_cmd;
    bus_read read;
} DevQSPIW25QxxObj_TypeDef;

typedef struct
{
    bool (*init)(DevQSPIW25QxxObj_TypeDef *obj);
    bool (*read_sector)(DevQSPIW25QxxObj_TypeDef *obj, uint32_t addr, uint8_t *p_rx, uint16_t len);
    bool (*write_sector)(DevQSPIW25QxxObj_TypeDef *obj, uint32_t addr, uint8_t *p_tx, uint16_t len);
    bool (*erase_sector)(DevQSPIW25QxxObj_TypeDef *obj, uint32_t addr);
    bool (*erase_chip)(DevQSPIW25QxxObj_TypeDef *obj);
} DevQSPIW25Qxx_TypeDef;

extern DevQSPIW25Qxx_TypeDef DevQSPIW25Qxx;

#endif
