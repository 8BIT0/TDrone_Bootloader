#ifndef __HW_DEF_H
#define __HW_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "Bsp_GPIO.h"
#include "Bsp_DMA.h"
#include "Bsp_SPI.h"
#include "Bsp_Uart.h"
#include "Bsp_Flash.h"
#include "debug_util.h"
#include "Dev_Led.h"
#include "../../../../FCHW_Config.h"

extern const uint8_t HWVer[3];

#define LED1_PIN GPIO_PIN_7
#define LED1_PORT GPIOH

/* SPI3 Reserve SPI */
#define RESERVE_SPI_CLK_PORT GPIOB
#define RESERVE_SPI_CLK_PIN GPIO_PIN_3

#define RESERVE_SPI_MISO_PORT GPIOB
#define RESERVE_SPI_MISO_PIN GPIO_PIN_4

#define RESERVE_SPI_MOSI_PORT GPIOB
#define RESERVE_SPI_MOSI_PIN GPIO_PIN_5

/* QSPI Port */
#define QSPI_CLK_PORT GPIOB
#define QSPI_NCS_PORT GPIOB
#define QSPI_BK_IO0_PORT GPIOD
#define QSPI_BK_IO1_PORT GPIOD
#define QSPI_BK_IO2_PORT GPIOE
#define QSPI_BK_IO3_PORT GPIOD

/* QSPI PIN */
#define QSPI_CLK_PIN GPIO_PIN_2
#define QSPI_NCS_PIN GPIO_PIN_6
#define QSPI_BK_IO0_PIN GPIO_PIN_11
#define QSPI_BK_IO1_PIN GPIO_PIN_12
#define QSPI_BK_IO2_PIN GPIO_PIN_2
#define QSPI_BK_IO3_PIN GPIO_PIN_13

/* QSPI Alternate */
#define QSPI_CLK_Alt GPIO_AF9_QUADSPI
#define QSPI_NCS_ALT GPIO_AF10_QUADSPI
#define QSPI_BK_IO0_ALT GPIO_AF9_QUADSPI
#define QSPI_BK_IO1_ALT GPIO_AF9_QUADSPI
#define QSPI_BK_IO2_ALT GPIO_AF9_QUADSPI
#define QSPI_BK_IO3_ALT GPIO_AF9_QUADSPI

/* Serial Pin */
#define UART4_TX_PORT GPIOB
#define UART4_TX_PIN GPIO_PIN_9
#define UART4_RX_PORT GPIOB
#define UART4_RX_PIN GPIO_PIN_8

#define UART1_TX_PORT GPIOA
#define UART1_TX_PIN GPIO_PIN_9
#define UART1_RX_PORT GPIOA
#define UART1_RX_PIN GPIO_PIN_10

/* USB Detected Pin */
#define USB_DETECT_INT_PORT GPIOE
#define USB_DETECT_INT_PIN GPIO_PIN_2

/* PWM IO */
#define PWM_SIG_1_TIM TIM3
#define PWM_SIG_1_TIM_CHANNEL TIM_CHANNEL_3
#define PWM_SIG_1_PORT GPIOB
#define PWM_SIG_1_PIN GPIO_PIN_0
#define PWM_SIG_1_DMA Bsp_DMA_1
#define PWM_SIG_1_DMA_CHANNEL Bsp_DMA_Stream_0
#define PWM_SIG_1_PIN_AF GPIO_AF2_TIM3

#define PWM_SIG_2_TIM TIM3
#define PWM_SIG_2_TIM_CHANNEL TIM_CHANNEL_4
#define PWM_SIG_2_PORT GPIOB
#define PWM_SIG_2_PIN GPIO_PIN_1
#define PWM_SIG_2_DMA Bsp_DMA_1
#define PWM_SIG_2_DMA_CHANNEL Bsp_DMA_Stream_1
#define PWM_SIG_2_PIN_AF GPIO_AF2_TIM3

#define PWM_SIG_3_TIM TIM5
#define PWM_SIG_3_TIM_CHANNEL TIM_CHANNEL_1
#define PWM_SIG_3_PORT GPIOA
#define PWM_SIG_3_PIN GPIO_PIN_0
#define PWM_SIG_3_DMA Bsp_DMA_1
#define PWM_SIG_3_DMA_CHANNEL Bsp_DMA_Stream_2
#define PWM_SIG_3_PIN_AF GPIO_AF2_TIM5

#define RECEIVER_PORT UART4
#define RECEIVER_CRSF_RX_DMA Bsp_DMA_None               // Bsp_DMA_1
#define RECEIVER_CRSF_RX_DMA_STREAM Bsp_DMA_Stream_None // Bsp_DMA_Stream_4
#define RECEIVER_CRSF_TX_DMA Bsp_DMA_None               // Bsp_DMA_1
#define RECEIVER_CRSF_TX_DMA_STREAM Bsp_DMA_Stream_None // Bsp_DMA_Stream_5

#if (FLASH_CHIP_STATE == ON)
#define ExtFlash_Bus_Type Storage_ChipBus_Spi
#define ExtFlash_Bus_Clock_Div SPI_MCLK_DIV_4
#define ExtFlash_Chip_Type Storage_ChipType_W25Qxx
#define ExtFlash_Bus_Api BspSPI
#define ExtFLash_Bus_Instance (void *)SPI2
#define ExtFlash_Bus_CLKPhase SPI_CLOCK_PHASE_2EDGE
#define ExtFlash_Bus_CLKPolarity SPI_CLOCK_POLARITY_HIGH
#define ExtFlash_CS_Pin ExtFlash_CSPin
#define ExtFlash_Bus_Pin ExtFlash_SPIPin

#define App_Firmware_Addr W25QXX_BASE_ADDRESS
#define App_Firmware_Size (1 Mb)

#define ExtFlash_Dev_Api (void *)(&DevW25Qxx)
#define ExtFlash_Start_Addr (App_Firmware_Addr + App_Firmware_Size)

#define ExtFlash_Storage_DefaultData FLASH_DEFAULT_DATA
#define ExtFlash_Storage_TotalSize (512 Kb)
#define ExtFlash_Storage_TabSize Flash_Storage_TabSize
#define ExtFlash_Storage_InfoPageSize Flash_Storage_InfoPageSize
#define ExtFlash_Storage_Reserve_Size (1 Mb) - ExtFlash_Storage_TotalSize 

#define BlackBox_Storage_Start_Addr (ExtFlash_Start_Addr + \
                                     ExtFlash_Storage_TotalSize + \
                                     ExtFlash_Storage_Reserve_Size)

/* store boot info boot parameter and firmware */
#define ExternalFlash_BootDataSec_Size (32 Kb)
#define ExternalFlash_SysDataSec_Size (64 Kb)
#define ExternalFlash_UserDataSec_Size (64 Kb)

extern BspGPIO_Obj_TypeDef ExtFlash_CSPin;
extern BspSPI_PinConfig_TypeDef ExtFlash_SPIPin;
#else
#define ExtFlash_Bus_Type Storage_ChipBus_None

#define App_Firmware_Addr 0
#define App_Firmware_Size (0 Mb)

#define ExtFlash_Dev_Api NULL
#define ExtFlash_Start_Addr (App_Firmware_Addr + App_Firmware_Size)

#define ExtFlash_Storage_DefaultData FLASH_DEFAULT_DATA
#define ExtFlash_Storage_TotalSize (0 Kb)
#define ExtFlash_Storage_TabSize Flash_Storage_TabSize
#define ExtFlash_Storage_InfoPageSize Flash_Storage_InfoPageSize
#define ExtFlash_Storage_Reserve_Size (0 Mb) - ExtFlash_Storage_TotalSize 

#define BlackBox_Storage_Start_Addr (ExtFlash_Start_Addr + \
                                     ExtFlash_Storage_TotalSize + \
                                     ExtFlash_Storage_Reserve_Size)

/* store boot info boot parameter and firmware */
#define ExternalFlash_BootDataSec_Size (0 Kb)
#define ExternalFlash_SysDataSec_Size (0 Kb)
#define ExternalFlash_UserDataSec_Size (0 Kb)
#endif

extern DebugPinObj_TypeDef Debug_PC0;
extern DebugPinObj_TypeDef Debug_PC1;
extern DebugPinObj_TypeDef Debug_PC2;
extern DebugPinObj_TypeDef Debug_PC3;
extern DebugPinObj_TypeDef Debug_PB3;
extern DebugPinObj_TypeDef Debug_PB4;
extern DebugPinObj_TypeDef Debug_PB5;
extern DebugPinObj_TypeDef Debug_PB6;
extern DebugPinObj_TypeDef Debug_PB10;

extern DevLedObj_TypeDef Led1;

extern BspGPIO_Obj_TypeDef USB_DctPin;
extern BspGPIO_Obj_TypeDef Uart4_TxPin;
extern BspGPIO_Obj_TypeDef Uart4_RxPin;
extern BspGPIO_Obj_TypeDef Uart1_TxPin;
extern BspGPIO_Obj_TypeDef Uart1_RxPin;

extern DebugPrintObj_TypeDef DebugPort;

#define DEBUG_TAG "[ DEBUG INFO ] "
#define DEBUG_INFO(fmt, ...) Debug_Print(&DebugPort, DEBUG_TAG, fmt, ##__VA_ARGS__)

#define Select_Hardware "Hardware NEURE"

#define Sample_Blinkly Led1
#define Noti_LED_Ptr NULL
#define BlackBox_Noti_Ptr NULL

#ifdef __cplusplus
}
#endif

#endif
