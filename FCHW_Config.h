#ifndef __FCHW_CONFIG_H
#define __FCHW_CONFIG_H

#include "util.h"

#define ON 1
#define OFF 0

#define Storage_ChipBus_None    0
#define Storage_ChipBus_Spi     (Storage_ChipBus_None + 1)
#define Storage_ChipBus_QSpi    (Storage_ChipBus_Spi + 1)

#define SDRAM_ENABLE_STATE      ON
#define FLASH_CHIP_ENABLE_STATE OFF
#define RADIO_NUM               1

#define Storage_InfoPageSize Flash_Storage_InfoPageSize

/* get virable from .ld file defined */
extern uint32_t __rom_s;
extern uint32_t __rom_e;
extern uint32_t __boot_s;
extern uint32_t __boot_e;
extern uint32_t __app_s;
extern uint32_t __app_e;
extern uint32_t __sdram_s1_s;
extern uint32_t __sdram_s1_e;

#define Boot_Address_Base   ((uint32_t)(&__boot_s))
#define Boot_Section_Size   ((uint32_t)&__boot_e - (uint32_t)&__boot_s)

#define App_Address_Base    ((uint32_t)&__app_s)
#define App_Section_Size    ((uint32_t)&__app_e - (uint32_t)&__app_s)

#define SDRAM_EN            SDRAM_ENABLE_STATE
#define SD_CARD             SD_CARD_ENABLE_STATE
#define FLASH_CHIP_STATE    FLASH_CHIP_ENABLE_STATE
#define RADIO_UART_NUM      RADIO_NUM

#define FC_SDRAM_Base_Addr  ((uint32_t)(&__sdram_s1_s))
#define FC_SDRAM_Size       ((uint32_t)(&__sdram_s1_s) - (uint32_t)(&__sdram_s1_e))

#endif
