#include "Bsp_Flash.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_flash.h"

#define BSP_FLASH_ADDR_ALIGN_SIZE 4
#define BSP_FLASH_WRITE_UNIT 32 /* unit : byte */

/* internal function */
static bool BspFlash_Get_Sector(uint32_t addr, uint32_t *p_bank, uint32_t *p_sector);

/* external function */
static bool BspFlash_Init(void);
static void BspFlash_DeInit(void);
static bool BspFlash_Erase(uint32_t addr, uint32_t len);
static bool BspFlash_Read_From_Addr(uint32_t addr, uint8_t *p_data, uint32_t size);
static bool BspFlash_Write_To_Addr(uint32_t addr, uint8_t *p_data, uint32_t size);

BspFlash_TypeDef BspFlash = {
    .init = BspFlash_Init,
    .de_init = BspFlash_DeInit,
    .erase = BspFlash_Erase,
    .read = BspFlash_Read_From_Addr,
    .write = BspFlash_Write_To_Addr,
};

static bool BspFlash_Init(void)
{
    FLASH_OBProgramInitTypeDef ob_init;
    uint32_t bank_wrp_status = 0xFFF;
    
    /* Allow Access to option bytes sector */
    HAL_FLASH_OB_Unlock();
    
    /* Allow Access to Flash control registers and user Flash */
    HAL_FLASH_Unlock();
    
    /* wait till flash idle */
    while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY));
    
    __HAL_FLASH_CLEAR_FLAG_BANK1(FLASH_FLAG_EOP_BANK1);
    __HAL_FLASH_CLEAR_FLAG_BANK1(FLASH_FLAG_EOP_BANK2);
        
    __HAL_FLASH_CLEAR_FLAG_BANK1(FLASH_FLAG_ALL_ERRORS_BANK1);
    __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_ALL_ERRORS_BANK2);

    /* Disable FLASH_WRP_SECTORS write protection */
    ob_init.OptionType = OPTIONBYTE_WRP;
    ob_init.Banks      = FLASH_BANK_1;
    ob_init.WRPState   = OB_WRPSTATE_DISABLE;
    ob_init.WRPSector  = OB_WRP_SECTOR_ALL;
    HAL_FLASHEx_OBProgram(&ob_init);
    /* Start the Option Bytes programming process */
    if (HAL_FLASH_OB_Launch() != HAL_OK)
        return false;
    
    /* Check if FLASH_WRP_SECTORS write protection is disabled */
    ob_init.Banks = FLASH_BANK_1;
    HAL_FLASHEx_OBGetConfig(&ob_init);
    bank_wrp_status = ob_init.WRPSector & OB_WRP_SECTOR_ALL;
    if (bank_wrp_status)
        /* Write  protection is not disabled */
        return false;
    
    /* Disable FLASH_WRP_SECTORS write protection */
    ob_init.OptionType = OPTIONBYTE_WRP;
    ob_init.Banks      = FLASH_BANK_2;
    ob_init.WRPState   = OB_WRPSTATE_DISABLE;
    ob_init.WRPSector  = OB_WRP_SECTOR_ALL;
    HAL_FLASHEx_OBProgram(&ob_init);
    /* Start the Option Bytes programming process */
    if (HAL_FLASH_OB_Launch() != HAL_OK)
        return false;
    
    /* Check if FLASH_WRP_SECTORS write protection is disabled */
    ob_init.Banks = FLASH_BANK_2;
    HAL_FLASHEx_OBGetConfig(&ob_init);
    bank_wrp_status = ob_init.WRPSector & OB_WRP_SECTOR_ALL;
    if (bank_wrp_status)
        /* Write  protection is not disabled */
        return false;
    
    HAL_FLASH_Lock();

    /* Prevent Access to option bytes sector */
    HAL_FLASH_OB_Lock();
    
    return true;
}

static void BspFlash_DeInit(void)
{
    HAL_FLASH_Lock();
    HAL_FLASH_OB_Lock();
}

static bool BspFlash_Read_From_Addr(uint32_t addr, uint8_t *p_data, uint32_t size)
{
    if ((p_data == NULL) || (addr < FLASH_BASE_ADDR) || \
        (addr + size >= FLASH_BASE_ADDR + FLASH_SIZE))
        return false;

    while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY));

    SCB_DisableICache();
    for(uint32_t i = 0; i < (size / 4); i++)
    {
        ((uint32_t *)p_data)[i] = ((volatile uint32_t *)addr)[i];
        __DSB();
    }
    SCB_EnableICache();

    return true;
}

static bool BspFlash_Write_To_Addr(uint32_t addr, uint8_t *p_data, uint32_t size)
{
    uint8_t buff_tmp[BSP_FLASH_WRITE_UNIT] = {0};
    uint16_t write_len = 0;

    if ((addr <= FLASH_BASE_ADDR) || \
        (addr >= FLASH_END) || \
        (p_data == NULL) || (size == 0) || \
        (HAL_FLASH_Unlock() != HAL_OK))
        return false;

    /* check bank */
    if (((addr & 0x0FF00000UL) == FLASH_BANK1_BASE) && \
        ((addr + size) >= (FLASH_BANK2_BASE - 1)))
    {
        /* bank 1 */
        return false;
    }
    else if (((addr & 0x0FF00000UL) == FLASH_BANK2_BASE) && \
             ((addr + size) >= FLASH_END))
    {
        /* bank 2 */
        return false;
    }

    for (uint32_t i = 0; i < size; i += 32)
    {
        write_len = (size - i) < 32 ? (size - i) : 32;
        if (32 == write_len)
        {
            /* program 32 byte */
            if (HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr + i, (uint32_t)p_data + i))
            {
                HAL_FLASH_Lock();
                return false;
            }
            
            /* program check */
            BspFlash_Read_From_Addr(addr + i, buff_tmp, 32);
            if (0 != memcmp(p_data + i, buff_tmp, 32))
            {
                HAL_FLASH_Lock();
                return false;
            }
        }
        else
        {
            BspFlash_Read_From_Addr(addr + i, buff_tmp, 32);
            memcpy(buff_tmp, p_data + i, write_len);
            
            if (HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr + i, (uint32_t)buff_tmp))
            {
                HAL_FLASH_Lock();
                return 0;
            }
            
            BspFlash_Read_From_Addr(addr + i, buff_tmp, write_len);
            if (0 != memcmp(p_data + i, buff_tmp, write_len))
            {
                HAL_FLASH_Lock();
                return false;
            }
        }
    }
    
    HAL_FLASH_Lock();
    return true;
}

static bool BspFlash_Erase(uint32_t addr, uint32_t len)
{
    uint32_t PageError = 0;
    uint8_t sector_number;
    uint32_t erase_addr, erase_len;
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef eraseinitstruct;
    
    /* 参数检查 */
    if ((addr < FLASH_BASE_ADDR) || \
        (addr + len >= FLASH_BASE_ADDR + FLASH_SIZE) || \
        ((addr - FLASH_BASE_ADDR) % FLASH_SECTOR_SIZE) || \
        (HAL_FLASH_Unlock() != HAL_OK))
        return false;
    
    erase_addr = addr;
    erase_len = len;
    
    if (erase_addr < FLASH_BANK2_BASE && erase_addr + erase_len >= FLASH_BANK2_BASE)
    {
        erase_len = FLASH_BANK2_BASE - erase_addr;
        sector_number = erase_len / FLASH_SECTOR_SIZE;
        if (0 != erase_len % FLASH_SECTOR_SIZE)
            sector_number += 1;
        
        if (0 == BspFlash_Get_Sector(erase_addr, &eraseinitstruct.Banks, &eraseinitstruct.Sector))
        {
            HAL_FLASH_Lock();
            return false;
        }

        eraseinitstruct.TypeErase = FLASH_TYPEERASE_SECTORS;
        eraseinitstruct.NbSectors = sector_number;
        eraseinitstruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
        
        status = HAL_FLASHEx_Erase(&eraseinitstruct, &PageError);
        
        if (status != HAL_OK)
        {
            HAL_FLASH_Lock();
            return false;
        }
        
        erase_addr = FLASH_BANK2_BASE;
        erase_len = len - erase_len;
    }
    
    sector_number = erase_len / FLASH_SECTOR_SIZE;
    if (0 != erase_len % FLASH_SECTOR_SIZE)
        sector_number += 1;
    
    if (0 == BspFlash_Get_Sector(erase_addr, &eraseinitstruct.Banks, &eraseinitstruct.Sector))
    {
        HAL_FLASH_Lock();
        return false;
    }

    eraseinitstruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseinitstruct.NbSectors = sector_number;

    return true;
}

static bool BspFlash_Get_Sector(uint32_t addr, uint32_t *p_bank, uint32_t *p_sector)
{
    if(addr)
    {
        if ((addr < FLASH_BASE_ADDR) || \
            (NULL == p_bank) || \
            (NULL == p_sector))
            return false;
        
        /* check bank */
        if ((addr & 0x0FF00000UL) == FLASH_BANK1_BASE)
        {
            if (addr >= FLASH_BANK1_BASE + FLASH_SIZE)
                return false;

            /* bank 1 */
            *p_bank = FLASH_BANK_1;
            addr -= FLASH_BANK1_BASE;
        }
        else if ((addr & 0x0FF00000UL) == FLASH_BANK2_BASE)
        {
            if (addr >= FLASH_BANK2_BASE + FLASH_SIZE)
                return false;

            /* bank 2 */
            *p_bank = FLASH_BANK_2;
            addr -= FLASH_BANK2_BASE;
        }
        else
            return false;
        
        if (addr < FLASH_SECTOR_1_OFFSET_ADDR)
        {
            *p_sector = FLASH_SECTOR_0;
        }
        else if ((addr >= FLASH_SECTOR_1_OFFSET_ADDR) && (addr < FLASH_SECTOR_2_OFFSET_ADDR))
        {
            *p_sector = FLASH_SECTOR_1;
        }
        else if ((addr >= FLASH_SECTOR_2_OFFSET_ADDR) && (addr < FLASH_SECTOR_3_OFFSET_ADDR))
        {
            *p_sector = FLASH_SECTOR_2;
        }
        else if ((addr >= FLASH_SECTOR_3_OFFSET_ADDR) && (addr < FLASH_SECTOR_4_OFFSET_ADDR))
        {
            *p_sector = FLASH_SECTOR_3;
        }
        else if ((addr >= FLASH_SECTOR_4_OFFSET_ADDR) && (addr < FLASH_SECTOR_5_OFFSET_ADDR))
        {
            *p_sector = FLASH_SECTOR_4;
        }
        else if ((addr >= FLASH_SECTOR_5_OFFSET_ADDR) && (addr < FLASH_SECTOR_6_OFFSET_ADDR))
        {
            *p_sector = FLASH_SECTOR_5;
        }
        else if ((addr >= FLASH_SECTOR_6_OFFSET_ADDR) && (addr < FLASH_SECTOR_7_OFFSET_ADDR))
        {
            *p_sector = FLASH_SECTOR_6;
        }
        else if ((addr >= FLASH_SECTOR_7_OFFSET_ADDR) && (addr < FLASH_SECTOR_7_OFFSET_ADDR + FLASH_SECTOR_SIZE))
        {
            *p_sector = FLASH_SECTOR_7;
        }

        return true;
    }

    return false;
}
