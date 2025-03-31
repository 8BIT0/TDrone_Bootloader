#include "HW_Def.h"

const uint8_t HWVer[3] = {0, 0, 5};

DebugPinObj_TypeDef Debug_PC0 = {
    .port = GPIOC,
    .pin = GPIO_PIN_0,
    .init_state = false,
};

DebugPinObj_TypeDef Debug_PC1 = {
    .port = GPIOC,
    .pin = GPIO_PIN_1,
    .init_state = false,
};

DebugPinObj_TypeDef Debug_PC2 = {
    .port = GPIOC,
    .pin = GPIO_PIN_2,
    .init_state = false,
};

DebugPinObj_TypeDef Debug_PC3 = {
    .port = GPIOC,
    .pin = GPIO_PIN_3,
    .init_state = false,
};

DebugPinObj_TypeDef Debug_PB3 = {
    .port = GPIOB,
    .pin = GPIO_PIN_3,
    .init_state = false,
};

DebugPinObj_TypeDef Debug_PB4 = {
    .port = GPIOB,
    .pin = GPIO_PIN_4,
    .init_state = false,
};

DebugPinObj_TypeDef Debug_PB5 = {
    .port = GPIOB,
    .pin = GPIO_PIN_5,
    .init_state = false,
};

DebugPinObj_TypeDef Debug_PB6 = {
    .port = GPIOB,
    .pin = GPIO_PIN_6,
    .init_state = false,
};

DebugPinObj_TypeDef Debug_PB10 = {
    .port = GPIOB,
    .pin = GPIO_PIN_10,
    .init_state = false,
};

DevLedObj_TypeDef Led1 = {
    .port = LED1_PORT,
    .pin = LED1_PIN,
    .init_state = true,
};

BspGPIO_Obj_TypeDef Uart4_TxPin = {
    .pin = UART4_TX_PIN,
    .port = UART4_TX_PORT,
    .alternate = GPIO_AF8_UART4,
};

BspGPIO_Obj_TypeDef Uart4_RxPin = {
    .pin = UART4_RX_PIN,
    .port = UART4_RX_PORT,
    .alternate = GPIO_AF8_UART4,
};

BspGPIO_Obj_TypeDef Uart1_TxPin = {
    .pin = UART1_TX_PIN,
    .port = UART1_TX_PORT,
    .alternate = GPIO_AF7_USART1,
};

BspGPIO_Obj_TypeDef Uart1_RxPin = {
    .pin = UART1_RX_PIN,
    .port = UART1_RX_PORT,
    .alternate = GPIO_AF7_USART1,
};

BspGPIO_Obj_TypeDef USB_DctPin = {
    .init_state = false,
    .pin = USB_DETECT_INT_PIN,
    .port = USB_DETECT_INT_PORT,
};

void BspSDRAM_Pin_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct ={0};
  
    /** FMC GPIO Configuration
    PF0   ------> FMC_A0        PD14   ------> FMC_D0       PH5    ------> FMC_SDNWE
    PF1   ------> FMC_A1        PD15   ------> FMC_D1       PG15   ------> FMC_SDNCAS 
    PF2   ------> FMC_A2        PD0    ------> FMC_D2       PF11   ------> FMC_SDNRAS
    PF3   ------> FMC_A3        PD1    ------> FMC_D3       PH3    ------> FMC_SDNE0  
    PF4   ------> FMC_A4        PE7    ------> FMC_D4       PG4    ------> FMC_BA0
    PF5   ------> FMC_A5        PE8    ------> FMC_D5       PG5    ------> FMC_BA1
    PF12  ------> FMC_A6        PE9    ------> FMC_D6       PH2    ------> FMC_SDCKE0 
    PF13  ------> FMC_A7        PE10   ------> FMC_D7       PG8    ------> FMC_SDCLK
    PF14  ------> FMC_A8        PE11   ------> FMC_D8       PE1    ------> FMC_NBL1
    PF15  ------> FMC_A9        PE12   ------> FMC_D9       PE0    ------> FMC_NBL0
    PG0   ------> FMC_A10       PE13   ------> FMC_D10
    PG1   ------> FMC_A11       PE14   ------> FMC_D11
    PG2   ------> FMC_A12       PE15   ------> FMC_D12
                                PD8    ------> FMC_D13
                                PD9    ------> FMC_D14
                                PD10   ------> FMC_D15
    */

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                            |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_11|GPIO_PIN_12
                            |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4
                            |GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                            |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                            |GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14
                            |GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}


SPI_HandleTypeDef Baro_Bus_Instance;
BspGPIO_Obj_TypeDef *p_Baro_CS = NULL;

/* debug print port uart4 */
#define DEBUG_PORT_BAUDRATE 460800
#define DEBUG_P4_PORT UART4
#define DEBUG_P4_TX_PIN_INIT_STATE GPIO_NOPULL
#define DEBUG_P4_RX_PIN_INIT_STATE GPIO_NOPULL
#define DEBUG_P4_TX_PIN_ALT GPIO_AF6_UART4
#define DEBUG_P4_RX_PIN_ALT GPIO_AF6_UART4
#define DEBUG_P4_TX_DMA Bsp_DMA_None
#define DEBUG_P4_TX_DMA_STREAM Bsp_DMA_Stream_None
#define DEBUG_P4_RX_DMA Bsp_DMA_None
#define DEBUG_P4_RX_DMA_STREAM Bsp_DMA_Stream_None
#define DEBUG_P4_TX_PIN UART4_TX_PIN
#define DEBUG_P4_RX_PIN UART4_RX_PIN
#define DEBUG_P4_TX_PORT UART4_TX_PORT
#define DEBUG_P4_RX_PORT UART4_RX_PORT

static BspUARTObj_TypeDef Debug_Port4_Obj = {
    .instance = DEBUG_P4_PORT,
    .baudrate = DEBUG_PORT_BAUDRATE,
    .tx_io = {
        .init_state = DEBUG_P4_TX_PIN_INIT_STATE,
        .pin        = DEBUG_P4_TX_PIN,
        .port       = DEBUG_P4_TX_PORT,
        .alternate  = DEBUG_P4_TX_PIN_ALT,
    }, 
    .rx_io = {
        .init_state = DEBUG_P4_RX_PIN_INIT_STATE,
        .pin        = DEBUG_P4_RX_PIN,
        .port       = DEBUG_P4_RX_PORT,
        .alternate  = DEBUG_P4_RX_PIN_ALT,
    }, 
    .pin_swap   = false,
    .rx_dma     = DEBUG_P4_RX_DMA,
    .rx_stream  = DEBUG_P4_RX_DMA_STREAM,
    .tx_dma     = DEBUG_P4_TX_DMA,
    .tx_stream  = DEBUG_P4_TX_DMA_STREAM,
    .rx_buf     = NULL,
    .rx_size    = 0,
};

DebugPrintObj_TypeDef DebugPort = {
    .port_obj = &Debug_Port4_Obj,
};
