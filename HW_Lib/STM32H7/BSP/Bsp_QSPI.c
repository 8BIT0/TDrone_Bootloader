#include "Bsp_QSPI.h"
#include "Bsp_GPIO.h"
#include "stm32h7xx_hal.h"

#define DATALINE_0  ((uint8_t) 0)
#define DATALINE_1  ((uint8_t) 1)
#define DATALINE_2  ((uint8_t) 2)
#define DATALINE_4  ((uint8_t) 4)

typedef struct
{
    bool valid;
    QSPI_CommandTypeDef command;
} BspQPI_CMDInfo_TypeDef;

/* internal function */
static BspQPI_CMDInfo_TypeDef BspQSPI_Check_CMD(uint8_t data_line, uint8_t addr_line, uint32_t addr, uint32_t dummy_cyc, uint16_t size, uint32_t code);

/* external function */
static bool BspQSPI_Init(BspQSPI_Config_TypeDef *obj, void *qspi_hdl);
static bool BspQSPI_Command(BspQSPI_Config_TypeDef *obj, uint32_t data_line, uint32_t addr_line, uint32_t addr, uint32_t dummy_cyc, uint32_t size, uint32_t code);
static bool BspQSPI_Polling(BspQSPI_Config_TypeDef *obj, uint32_t data_line, uint32_t addr_line, uint32_t addr, uint32_t dummy_cyc, uint32_t size, uint32_t code, uint32_t match, uint32_t mask);
static bool BspQSPI_MemMap(BspQSPI_Config_TypeDef *obj, uint32_t cmd);
static bool BspQSPI_Recv(BspQSPI_Config_TypeDef *obj, uint8_t *p_data);
static bool BspQSPI_Trans(BspQSPI_Config_TypeDef *obj, uint8_t *p_data);

BspQSpi_TypeDef BspQspi = {
    .init    = BspQSPI_Init,
    .cmd     = BspQSPI_Command,
    .polling = BspQSPI_Polling,
    .memmap  = BspQSPI_MemMap,
    .rx      = BspQSPI_Recv,
    .tx      = BspQSPI_Trans,
};

__attribute__((weak)) void BspQSPI_Pin_Init(void) { return; }

static bool BspQSPI_Init(BspQSPI_Config_TypeDef *obj, void *qspi_hdl)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    if ((obj == NULL) || (qspi_hdl == NULL))
        return false;
    
    obj->init_state = false;
    obj->p_qspi = qspi_hdl;

    /* clock init */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_QSPI;
    PeriphClkInitStruct.QspiClockSelection = RCC_QSPICLKSOURCE_D1HCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
        return false;

    BspQSPI_Pin_Init();

    __HAL_RCC_QSPI_CLK_ENABLE();

    if (HAL_QSPI_DeInit(To_QSPI_Handle_Ptr(obj->p_qspi)) != HAL_OK)
        return false;
    
    To_QSPI_Handle_Ptr(obj->p_qspi)->Init.ClockPrescaler     = 1;
    To_QSPI_Handle_Ptr(obj->p_qspi)->Init.FifoThreshold      = 32;
    To_QSPI_Handle_Ptr(obj->p_qspi)->Init.SampleShifting     = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
    To_QSPI_Handle_Ptr(obj->p_qspi)->Init.FlashSize          = 22;
    To_QSPI_Handle_Ptr(obj->p_qspi)->Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
    To_QSPI_Handle_Ptr(obj->p_qspi)->Init.ClockMode          = QSPI_CLOCK_MODE_3;
    To_QSPI_Handle_Ptr(obj->p_qspi)->Init.FlashID            = QSPI_FLASH_ID_1;
    To_QSPI_Handle_Ptr(obj->p_qspi)->Init.DualFlash          = QSPI_DUALFLASH_DISABLE;

    if (HAL_QSPI_Init(To_QSPI_Handle_Ptr(obj->p_qspi)) != HAL_OK)
        return false;

    obj->init_state = true;
    return true;
}

static bool BspQSPI_Trans(BspQSPI_Config_TypeDef *obj, uint8_t *p_data)
{
    if ((obj == NULL) || (obj->p_qspi == NULL) || !obj->init_state)
        return false;
    
    /* send command */
    if (HAL_QSPI_Transmit(obj->p_qspi, p_data, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) == HAL_OK)
        return true;

    return false;
}

static bool BspQSPI_Recv(BspQSPI_Config_TypeDef *obj, uint8_t *p_data)
{
    if ((obj == NULL) || (obj->p_qspi == NULL) || !obj->init_state)
        return false;

    if (HAL_QSPI_Receive(obj->p_qspi, p_data, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) == HAL_OK)
        return true;

    return false;
}

static bool BspQSPI_Command(BspQSPI_Config_TypeDef *obj, uint32_t data_line, uint32_t addr_line, uint32_t addr, uint32_t dummy_cyc, uint32_t size, uint32_t code)
{
    BspQPI_CMDInfo_TypeDef cmd;
    cmd = BspQSPI_Check_CMD(data_line, addr_line, addr, dummy_cyc, size, code);

    if ((obj == NULL) || (obj->p_qspi == NULL) || !obj->init_state || !cmd.valid)
        return false;

    if (HAL_QSPI_Command(obj->p_qspi, &cmd.command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
        return false;

    return true;
}
static bool BspQSPI_Polling(BspQSPI_Config_TypeDef *obj, uint32_t data_line, uint32_t addr_line, uint32_t addr, uint32_t dummy_cyc, uint32_t size, uint32_t code, uint32_t match, uint32_t mask)
{
	QSPI_AutoPollingTypeDef s_config;
    BspQPI_CMDInfo_TypeDef cmd;

    memset(&cmd, 0, sizeof(BspQPI_CMDInfo_TypeDef));
    memset(&s_config, 0, sizeof(QSPI_AutoPollingTypeDef));

    if ((obj == NULL) || (obj->p_qspi == NULL) || !obj->init_state)
        return false;

    cmd = BspQSPI_Check_CMD(data_line, addr_line, addr, dummy_cyc, size, code);
    if (!cmd.valid)
        return false;

    s_config.Match              = match;
    s_config.Mask               = mask;
    s_config.MatchMode          = QSPI_MATCH_MODE_AND;
    s_config.StatusBytesSize    = 1;
    s_config.Interval           = 0x10;
    s_config.AutomaticStop      = QSPI_AUTOMATIC_STOP_ENABLE;

    if (HAL_QSPI_AutoPolling(obj->p_qspi, &cmd.command, &s_config, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
        return false;

    return true;
}

static bool BspQSPI_MemMap(BspQSPI_Config_TypeDef *obj, uint32_t cmd)
{
	QSPI_CommandTypeDef s_command;
	QSPI_MemoryMappedTypeDef s_mem_mapped_cfg;

    memset(&s_command, 0, sizeof(QSPI_CommandTypeDef));
    memset(&s_mem_mapped_cfg, 0, sizeof(QSPI_MemoryMappedTypeDef));

    if ((obj == NULL) || (obj->p_qspi == NULL) || !obj->init_state)
        return false;

    s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
    s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
    s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
    s_command.AddressMode       = QSPI_ADDRESS_4_LINES;
    s_command.DataMode          = QSPI_DATA_4_LINES;
    s_command.DummyCycles       = 6;
    s_command.Instruction       = cmd;
	
	s_mem_mapped_cfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
	s_mem_mapped_cfg.TimeOutPeriod     = 0;

    if (HAL_QSPI_MemoryMapped(obj->p_qspi, &s_command, &s_mem_mapped_cfg) != HAL_OK)
        return false;

    return true;
}

static BspQPI_CMDInfo_TypeDef BspQSPI_Check_CMD(uint8_t data_line, uint8_t addr_line, uint32_t addr, uint32_t dummy_cyc, uint16_t size, uint32_t code)
{
    BspQPI_CMDInfo_TypeDef cmd;

    cmd.valid = false;

    cmd.command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.command.AddressSize       = QSPI_ADDRESS_24_BITS;
    cmd.command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.command.DdrMode           = QSPI_DDR_MODE_DISABLE;
    cmd.command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    cmd.command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    switch (data_line)
    {
        case DATALINE_0: cmd.command.DataMode = QSPI_DATA_NONE; break;
        case DATALINE_1: cmd.command.DataMode = QSPI_DATA_1_LINE; break;
        case DATALINE_2: cmd.command.DataMode = QSPI_DATA_2_LINES; break;
        case DATALINE_4: cmd.command.DataMode = QSPI_DATA_4_LINES; break;
        default: return cmd;
    }

    switch (addr_line)
    {
        case DATALINE_0: cmd.command.AddressMode = QSPI_ADDRESS_NONE; break;
        case DATALINE_1: cmd.command.AddressMode = QSPI_ADDRESS_1_LINE; break;
        case DATALINE_2: cmd.command.AddressMode = QSPI_ADDRESS_2_LINES; break;
        case DATALINE_4: cmd.command.AddressMode = QSPI_ADDRESS_4_LINES; break;
        default: return cmd;
    }

    cmd.command.Address     = addr;
    cmd.command.NbData      = size;
    cmd.command.DummyCycles = dummy_cyc;
    cmd.command.Instruction = code;

    cmd.valid = true;
    return cmd;
}
