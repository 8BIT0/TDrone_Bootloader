#include "Bsp_QSPI.h"
#include "Bsp_GPIO.h"
#include "stm32h7xx_hal.h"

/* external function */
static bool Bsp_QSPI_Init(BspQSPI_Config_TypeDef *obj);
static bool Bsp_QSPI_Command(BspQSPI_Config_TypeDef *obj, uint32_t mode, uint32_t dummy_cyc, uint32_t nb_data, uint32_t cmd);
static bool Bsp_QSPI_Polling(BspQSPI_Config_TypeDef *obj, uint32_t mode, uint32_t cmd, uint32_t cyc, uint32_t nb_data, uint32_t match, uint32_t mask);
static bool Bsp_QSPI_MemMap(BspQSPI_Config_TypeDef *obj, uint32_t cmd);
static bool Bsp_QSPI_Recv(BspQSPI_Config_TypeDef *obj, uint32_t addr, uint32_t cmd, uint8_t *p_data, uint32_t len);
static bool Bsp_QSPI_Trans(BspQSPI_Config_TypeDef *obj, uint32_t addr, uint32_t cmd, uint8_t *p_data, uint32_t len);

BspQSpi_TypeDef BspQspi = {
    .init    = Bsp_QSPI_Init,
    .cmd     = Bsp_QSPI_Command,
    .polling = Bsp_QSPI_Polling,
    .memmap  = Bsp_QSPI_MemMap,
    .rx      = Bsp_QSPI_Recv,
    .tx      = Bsp_QSPI_Trans,
};

static bool Bsp_QSPI_PinInit(BspQSPI_Config_TypeDef *obj)
{
    bool pin_state = false;
    BspGPIO_Obj_TypeDef pin_tmp;

    memset(&pin_tmp, 0, sizeof(BspGPIO_Obj_TypeDef));
    if ((obj == NULL) || \
        (obj->pin.port_clk == NULL) || \
        (obj->pin.port_ncs == NULL) || \
        (obj->pin.port_io0 == NULL) || \
        (obj->pin.port_io1 == NULL) || \
        (obj->pin.port_io2 == NULL) || \
        (obj->pin.port_io3 == NULL))
        return false;

    /* CLK */
    pin_tmp.port = obj->pin.port_clk;
    pin_tmp.pin = obj->pin.pin_clk;
    pin_tmp.alternate = obj->pin.alt_clk;
    pin_state = BspGPIO.alt_init(pin_tmp, GPIO_MODE_AF_PP);

    /* NCS */
    pin_tmp.port = obj->pin.port_ncs;
    pin_tmp.pin = obj->pin.pin_ncs;
    pin_tmp.alternate = obj->pin.alt_ncs;
    pin_state &= BspGPIO.alt_init(pin_tmp, GPIO_MODE_AF_PP);

    /* BK1_IO0 */
    pin_tmp.port = obj->pin.port_io0;
    pin_tmp.pin = obj->pin.pin_io0;
    pin_tmp.alternate = obj->pin.alt_io0;
    pin_state &= BspGPIO.alt_init(pin_tmp, GPIO_MODE_AF_PP);
    
    /* BK1_IO1 */
    pin_tmp.port = obj->pin.port_io1;
    pin_tmp.pin = obj->pin.pin_io1;
    pin_tmp.alternate = obj->pin.alt_io1;
    pin_state &= BspGPIO.alt_init(pin_tmp, GPIO_MODE_AF_PP);
    
    /* BK1_IO2 */
    pin_tmp.port = obj->pin.port_io2;
    pin_tmp.pin = obj->pin.pin_io2;
    pin_tmp.alternate = obj->pin.alt_io2;
    pin_state &= BspGPIO.alt_init(pin_tmp, GPIO_MODE_AF_PP);
    
    /* BK1_IO3 */
    pin_tmp.port = obj->pin.port_io3;
    pin_tmp.pin = obj->pin.pin_io3;
    pin_tmp.alternate = obj->pin.alt_io3;
    pin_state &= BspGPIO.alt_init(pin_tmp, GPIO_MODE_AF_PP);

    return pin_state;
}

static bool Bsp_QSPI_Init(BspQSPI_Config_TypeDef *obj)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    if ((obj == NULL) || \
        (obj->p_qspi == NULL))
        return false;
    
    obj->init_state = false;

    /* clock init */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_QSPI;
    PeriphClkInitStruct.QspiClockSelection = RCC_QSPICLKSOURCE_D1HCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
        return false;

    if (!Bsp_QSPI_PinInit(obj))
        return false;

    __HAL_RCC_QSPI_CLK_ENABLE();

    obj->p_qspi.Instance                = QUADSPI;
	HAL_QSPI_DeInit(obj->p_qspi);
	obj->p_qspi.Init.ClockPrescaler     = 1;
	obj->p_qspi.Init.FifoThreshold      = 32;
	obj->p_qspi.Init.SampleShifting     = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
	obj->p_qspi.Init.FlashSize          = 22;
	obj->p_qspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
	obj->p_qspi.Init.ClockMode          = QSPI_CLOCK_MODE_3;
	obj->p_qspi.Init.FlashID            = QSPI_FLASH_ID_1;
	obj->p_qspi.Init.DualFlash          = QSPI_DUALFLASH_DISABLE;
	HAL_QSPI_Init(obj->p_qspi);

    obj->init_state = true;
    return true;
}

static bool Bsp_QSPI_Trans(BspQSPI_Config_TypeDef *obj, uint32_t addr, uint32_t cmd, uint8_t *p_data, uint32_t len)
{
	QSPI_CommandTypeDef s_command;

    if ((obj == NULL) || \
        (obj->p_qspi == NULL) || \
        !obj->init_state)
        return false;

    s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
    s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
    s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
    s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
    s_command.DataMode          = QSPI_DATA_4_LINES;
    s_command.DummyCycles       = 0;
    s_command.NbData            = len;
    s_command.Address           = addr;
    s_command.Instruction       = cmd;
    
    /* send command */
    if ((HAL_QSPI_Command(obj->p_qspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) || \
        (HAL_QSPI_Transmit(obj->p_qspi, p_data, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK))
        return false;

    return true;
}

static bool Bsp_QSPI_Recv(BspQSPI_Config_TypeDef *obj, uint32_t addr, uint32_t cmd, uint8_t *p_data, uint32_t len)
{
	QSPI_CommandTypeDef s_command;

    if ((obj == NULL) || \
        (obj->p_qspi == NULL) || \
        !obj->init_state)
        return false;
	
	s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
	s_command.AddressMode 		= QSPI_ADDRESS_4_LINES;
	s_command.DataMode    		= QSPI_DATA_4_LINES;
	s_command.DummyCycles 		= 6;
	s_command.NbData      		= len;
	s_command.Address     		= addr;
	s_command.Instruction 		= cmd;

    if ((HAL_QSPI_Command(obj->p_qspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) || \
        (HAL_QSPI_Receive(obj->p_qspi, p_data, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK))
        return false;

    return true;
}

static bool Bsp_QSPI_Command(BspQSPI_Config_TypeDef *obj, uint32_t mode, uint32_t cyc, uint32_t nb_data, uint32_t cmd)
{
    QSPI_CommandTypeDef s_command;

    memset(&s_command, 0, sizeof(QSPI_CommandTypeDef));

	s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	s_command.AddressMode       = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
	s_command.DataMode          = mode;
    s_command.NbData            = nb_data;
	s_command.DummyCycles       = cyc;
	s_command.Instruction       = cmd;

    if ((obj == NULL) || \
        (obj->p_qspi == NULL) || \
        !obj->init_state)
        return false;

    if (HAL_QSPI_Command(obj->p_qspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
        return false;

    return true;
}

static bool Bsp_QSPI_Polling(BspQSPI_Config_TypeDef *obj, uint32_t mode, uint32_t cmd, uint32_t cyc, uint32_t nb_data, uint32_t match, uint32_t mask)
{
	QSPI_CommandTypeDef s_command;
	QSPI_AutoPollingTypeDef s_config;

    memset(&s_command, 0, sizeof(QSPI_CommandTypeDef));
    memset(&s_config, 0, sizeof(QSPI_AutoPollingTypeDef));

    if ((obj == NULL) || \
        (obj->p_qspi == NULL) || \
        !obj->init_state)
        return false;

	s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	s_command.AddressMode       = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
	s_command.DataMode          = mode;
	s_command.DummyCycles       = cyc;
	s_command.Instruction       = cmd;
    s_command.NbData            = nb_data;

	s_config.Match              = match;
	s_config.Mask               = mask;
	s_config.MatchMode          = QSPI_MATCH_MODE_AND;
	s_config.StatusBytesSize    = 1;
	s_config.Interval           = 0x10;
	s_config.AutomaticStop      = QSPI_AUTOMATIC_STOP_ENABLE;

    if (HAL_QSPI_AutoPolling(obj->p_qspi, &s_command, &s_config, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
        return false;

    return true;
}

static bool Bsp_QSPI_MemMap(BspQSPI_Config_TypeDef *obj, uint32_t cmd)
{
	QSPI_CommandTypeDef s_command;
	QSPI_MemoryMappedTypeDef s_mem_mapped_cfg;

    memset(&s_command, 0, sizeof(QSPI_CommandTypeDef));
    memset(&s_mem_mapped_cfg, 0, sizeof(QSPI_MemoryMappedTypeDef));

    if ((obj == NULL) || \
        (obj->p_qspi == NULL) || \
        !obj->init_state)
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
