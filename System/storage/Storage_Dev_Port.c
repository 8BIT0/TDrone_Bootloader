#include "Storage_Bus_Port.h"
#include "Storage_Dev_Port.h"
#include "Storage_Def.h"
#include "Srv_OsCommon.h"
#include "Dev_W25Qxx.h"
#include "Dev_W25Qxx_QSPI.h"

#include "HW_Def.h"
#include "debug_util.h"

#define Storage_Dev_GetSysTick  SrvOsCommon.get_os_ms
#define Storage_Dev_Malloc(x)   SrvOsCommon.malloc(x)
#define Storage_Dev_Free(x)     SrvOsCommon.free(x)

/* debug */
#define STORAGE_DEV_TAG                     "[ STORAGE DEV INFO ] "
#define STORAGE_DEV_INFO(fmt, ...)          Debug_Print(&DebugPort, STORAGE_DEV_TAG, fmt, ##__VA_ARGS__)

typedef DevNorFlash_Info_TypeDef (*dev_info)(void *obj);
typedef int8_t (*dev_init)(void *obj);
typedef int8_t (*dev_write)(void *obj, uint32_t addr, uint8_t *p_tx, uint16_t len);
typedef int8_t (*dev_read)(void *obj, uint32_t addr, uint8_t *p_rx, uint16_t len);
typedef int8_t (*dev_erase)(void *obj, uint32_t addr);
typedef int8_t (*dev_erase_chip)(void *obj);
typedef uint32_t (*dev_get_start_addr)(void *obj, uint32_t addr);

typedef struct
{
    dev_info info;
    dev_init init;
    dev_write write;
    dev_read read;
    dev_erase erase;
    dev_erase_chip erase_chip;
    dev_get_start_addr get_start_addr;
} StorageDev_TypeDef;

/* internal vriable */
static StorageDev_TypeDef Dev = {
    .info           = NULL,
    .init           = NULL,
    .erase          = NULL,
    .erase_chip     = NULL,
    .get_start_addr = NULL,
    .read           = NULL,
    .write          = NULL,
};
static uint8_t read_tmp[Storage_TabSize * 2] __attribute__((aligned(4))) __attribute__((section(".Perph_Section"))) = {0};
static uint8_t write_tmp[Storage_TabSize * 2] __attribute__((aligned(4))) __attribute__((section(".Perph_Section"))) = {0};

/* external function */
static bool Storage_Dev_Set(StorageDevObj_TypeDef *ext_dev);
static bool Storage_Dev_Init(StorageDevObj_TypeDef *ext_dev, uint32_t *p_code);

static bool Storage_Dev_Write_Section(StorageDevObj_TypeDef *p_dev, uint32_t addr, uint8_t *p_data, uint32_t len);
static bool Storage_Dev_Read_Section(StorageDevObj_TypeDef *p_dev, uint32_t addr, uint8_t *p_data, uint32_t len);
static bool Storage_Dev_Erase_Section(StorageDevObj_TypeDef *p_dev, uint32_t addr, uint32_t len);

static bool Storage_Dev_Param_Read(StorageDevObj_TypeDef *p_dev, uint32_t base_addr, uint8_t *p_data, uint32_t len);
static bool Storage_Dev_Param_Write(StorageDevObj_TypeDef *p_dev, uint32_t base_addr, uint8_t *p_data, uint32_t len);
static bool Storage_Dev_Param_Erase(StorageDevObj_TypeDef *p_dev, uint32_t base_addr, uint32_t len);

StorageDevApi_TypeDef StorageDev = {
    .set            = Storage_Dev_Set,
    .init           = Storage_Dev_Init,

    .write_phy_sec  = Storage_Dev_Write_Section,
    .read_phy_sec   = Storage_Dev_Read_Section,
    .erase_phy_sec  = Storage_Dev_Erase_Section,

    .param_read     = Storage_Dev_Param_Read,
    .param_write    = Storage_Dev_Param_Write,
    .param_erase    = Storage_Dev_Param_Erase,
};

static bool Storage_Dev_Set(StorageDevObj_TypeDef *ext_dev)
{
    if ((ext_dev == NULL) || \
        (ext_dev->api == NULL))
        return false;

    ext_dev->start_addr  = 0;
    ext_dev->sector_num  = 0;
    ext_dev->sector_size = 0;
    ext_dev->total_size  = 0;
    ext_dev->page_num    = 0;
    ext_dev->page_size   = 0;

    if (ext_dev->chip_type == Storage_ChipType_W25Qxx)
    {
        STORAGE_DEV_INFO(" W25Qxx selected\r\n");
#if (FLASH_CHIP_STATE == Storage_ChipBus_Spi)
        ext_dev->obj = Storage_Dev_Malloc(sizeof(DevW25QxxObj_TypeDef));
        if (ext_dev->obj == NULL)
            return false;

        To_DevW25Qxx_OBJ(ext_dev->obj)->systick   = Storage_Dev_GetSysTick;
        To_DevW25Qxx_OBJ(ext_dev->obj)->cs_ctl    = StoragePort_Api.cs_ctl;
        To_DevW25Qxx_OBJ(ext_dev->obj)->bus_tx    = StoragePort_Api.bus_tx;
        To_DevW25Qxx_OBJ(ext_dev->obj)->bus_rx    = StoragePort_Api.bus_rx;
        To_DevW25Qxx_OBJ(ext_dev->obj)->delay_ms  = SrvOsCommon.delay_ms;

#elif (FLASH_CHIP_STATE == Storage_ChipBus_QSpi)
        ext_dev->obj = Storage_Dev_Malloc(sizeof(DevQSPIW25QxxObj_TypeDef));
        if (ext_dev->obj == NULL)
            return false;
        
        To_DevQSPIW25Qxx_OBJ(ext_dev->obj)->init      = false;
        To_DevQSPIW25Qxx_OBJ(ext_dev->obj)->trans_cmd = StoragePort_Api.bus_trans_cmd;
        To_DevQSPIW25Qxx_OBJ(ext_dev->obj)->mem_map   = StoragePort_Api.bus_mem_map;
        To_DevQSPIW25Qxx_OBJ(ext_dev->obj)->polling   = StoragePort_Api.bus_status_polling;
        To_DevQSPIW25Qxx_OBJ(ext_dev->obj)->read      = StoragePort_Api.bus_read;
        To_DevQSPIW25Qxx_OBJ(ext_dev->obj)->write     = StoragePort_Api.bus_write;

        Dev.info            = (dev_info)To_DevQSPIW25Qxx_API(ext_dev->api)->info;
        Dev.init            = (dev_init)To_DevQSPIW25Qxx_API(ext_dev->api)->Init;
        Dev.erase           = (dev_erase)To_DevQSPIW25Qxx_API(ext_dev->api)->Erase_Sector;
        Dev.erase_chip      = (dev_erase_chip)To_DevQSPIW25Qxx_API(ext_dev->api)->Erase_Chip;
        Dev.read            = (dev_read)To_DevQSPIW25Qxx_API(ext_dev->api)->Read_Sector;
        Dev.write           = (dev_write)To_DevQSPIW25Qxx_API(ext_dev->api)->Write_Sector;
        Dev.get_start_addr  = (dev_get_start_addr)To_DevQSPIW25Qxx_API(ext_dev->api)->get_section_start_addr;
#endif
    }
    
    if ((Dev.info == NULL) || (Dev.init == NULL)  || (Dev.erase == NULL) || \
        (Dev.read == NULL) || (Dev.write == NULL) || (Dev.get_start_addr == NULL))
        return false;

    return true;
}

static bool Storage_Dev_Init(StorageDevObj_TypeDef *ext_dev, uint32_t *p_code)
{
    uint8_t init_state = 0;
    DevNorFlash_Info_TypeDef FlashInfo;

    if ((ext_dev == NULL) || (Dev.init == NULL) || (Dev.info == NULL))
        return false;

    memset(&FlashInfo, 0, sizeof(DevNorFlash_Info_TypeDef));
    STORAGE_DEV_INFO(" Module initializing\r\n");

    init_state = Dev.init(ext_dev->obj);
    FlashInfo  = Dev.info(ext_dev->obj);

    *p_code              = FlashInfo.prod_code;
    ext_dev->start_addr  = FlashInfo.start_addr;
    ext_dev->sector_num  = FlashInfo.sector_num;
    ext_dev->sector_size = FlashInfo.sector_size;
    ext_dev->total_size  = FlashInfo.flash_size;
    ext_dev->page_num    = FlashInfo.page_num;
    ext_dev->page_size   = FlashInfo.page_size;

    return (init_state == 0) ? true : false;
}

static bool Storage_Dev_Write_Section(StorageDevObj_TypeDef *p_dev, uint32_t addr, uint8_t *p_data, uint32_t len)
{
    uint32_t write_cnt = 0;
    uint32_t addr_tmp = 0;

    if ((p_dev == NULL) || \
        (p_dev->obj == NULL) || \
        (Dev.write == NULL) || \
        (Dev.erase == NULL) || \
        (p_data == NULL) || \
        (len == 0) || \
        ((addr % p_dev->sector_size) || \
        (len % p_dev->sector_size)))
        return false;

    write_cnt = len / p_dev->sector_size;
    addr_tmp = addr;

    for (uint8_t i = 0; i < write_cnt; i ++)
    {
        /* erase sector and update sector */
        if (Dev.erase(p_dev->obj, addr_tmp) | \
            Dev.write(p_dev->obj, addr_tmp, p_data, p_dev->sector_size))
            return false;

        addr_tmp += p_dev->sector_size;
    }

    return true;
}

static bool Storage_Dev_Read_Section(StorageDevObj_TypeDef *p_dev, uint32_t addr, uint8_t *p_data, uint32_t len)
{
    uint32_t read_cnt = 0;
    uint32_t addr_tmp = 0;
    
    if ((p_dev == NULL) || \
        (p_dev->obj == NULL) || \
        (Dev.read == NULL) || \
        (p_data == NULL) || \
        (len == 0) || \
        ((addr % p_dev->sector_size) || \
        (len % p_dev->sector_size)))
        return false;

    read_cnt = len / p_dev->sector_size;
    addr_tmp = addr;

    for (uint8_t i = 0; i < read_cnt; i++)
    {
        /* read sector */
        if (Dev.read(p_dev->obj, addr_tmp, p_data, len))
        {
            memset(p_data, 0, len);
            return false;
        }

        addr_tmp += p_dev->sector_size;
    }

    return true;
}

static bool Storage_Dev_Erase_Section(StorageDevObj_TypeDef *p_dev, uint32_t addr, uint32_t len)
{
    uint32_t erase_cnt = 0;
    uint32_t addr_tmp = 0;

    if ((p_dev == NULL) || \
        (Dev.erase == NULL) || \
        (p_dev->obj == NULL) || \
        (len == 0) || \
        ((addr % p_dev->sector_size) || \
        (len % p_dev->sector_size)))
        return false;

    erase_cnt = len / p_dev->sector_size;
    addr_tmp = addr;

    for (uint8_t i = 0; i < erase_cnt; i ++)
    {
        /* erase sector */
        if (Dev.erase(p_dev->obj, addr_tmp))
            return false;

        addr_tmp += p_dev->sector_size;
    }

    return true;
}

static bool Storage_Dev_Param_Read(StorageDevObj_TypeDef *p_dev, uint32_t base_addr, uint8_t *p_data, uint32_t len)
{
    uint32_t read_start_addr = base_addr;
    uint32_t flash_end_addr = 0;
    uint32_t section_start_addr = 0;
    uint32_t next_read_addr = 0;
    uint32_t section_size = 0;
    uint32_t read_offset = 0;
    uint32_t read_len = len;
    DevNorFlash_Info_TypeDef FlashInfo;

    if ((p_dev == NULL) || \
        (Dev.read == NULL) || \
        (Dev.info == NULL) || \
        (Dev.get_start_addr == NULL) || \
        (p_dev->obj == NULL) || \
        (sizeof(read_tmp) < len) || \
        (p_data == 0) || \
        (len == 0))
        return false;
    
    FlashInfo = Dev.info(To_DevW25Qxx_OBJ(p_dev->obj));

    section_size = FlashInfo.sector_size;
    /* get w25qxx device info */
    /* address check */
    flash_end_addr = FlashInfo.start_addr;
    if (flash_end_addr > read_start_addr)
        return false;

    /* range check */
    flash_end_addr += FlashInfo.flash_size;
    if ((len + read_start_addr) > flash_end_addr)
        return false;

    if (section_size == 0)
        return false;
        
    section_start_addr = Dev.get_start_addr(p_dev->obj, read_start_addr);
    read_offset = read_start_addr - section_start_addr;
    if (section_size > sizeof(read_tmp))
        return false;

    while(true)
    {
        /* circumstances 1: store data size less than flash sector size and only none multiple sector read is needed */
        /* circumstances 2: store data size less than flash sector length but need to read from the end of the sector N to the start of the sector N + 1 */
        /* circumstances 3: store data size large than flash sector length */
        if (read_offset + read_len > section_size)
            read_len = section_size - read_offset;

        /* read whole section */
        if (Dev.read(p_dev->obj, section_start_addr, read_tmp, section_size))
            return false;
    
        memcpy(p_data, read_tmp + read_offset, read_len);
        memset(read_tmp, 0, section_size);

        len -= read_len;
        if (len == 0)
            return true;
    
        read_offset = 0;
        next_read_addr = Dev.get_start_addr(p_dev->obj, section_start_addr + read_len);
        if (next_read_addr == section_start_addr)
            read_offset = read_len;
        
        p_data += read_len;
        read_len = len;
        section_start_addr = next_read_addr;
    }

    return false;
}

static bool Storage_Dev_Param_Write(StorageDevObj_TypeDef *p_dev, uint32_t base_addr, uint8_t *p_data, uint32_t len)
{
    uint32_t write_start_addr = base_addr;
    uint32_t flash_end_addr = 0;
    uint32_t section_start_addr = 0;
    uint32_t next_write_addr = 0;
    uint32_t section_size = 0;
    uint32_t write_offset = 0;
    uint32_t write_len = len;
    DevNorFlash_Info_TypeDef FlashInfo;

    if ((p_dev == NULL) || \
        (Dev.info == NULL) || \
        (Dev.get_start_addr == NULL) || \
        (Dev.read == NULL) || \
        (Dev.write == NULL) || \
        (Dev.erase == NULL) || \
        (p_dev->obj == NULL) || \
        (p_data == NULL) || \
        (len == 0))
        return false;

    memset(&FlashInfo, 0, sizeof(DevNorFlash_Info_TypeDef));
    FlashInfo = Dev.info(p_dev->obj);

    section_size = FlashInfo.sector_size;
    /* get w25qxx device info */
    /* address check */
    flash_end_addr = FlashInfo.start_addr;
    if (flash_end_addr > write_start_addr)
        return false;

    /* range check */
    flash_end_addr += FlashInfo.flash_size;
    if ((len + write_start_addr) > flash_end_addr)
        return false;
    
    if (section_size == 0)
        return false;
        
    section_start_addr = Dev.get_start_addr(p_dev->obj, write_start_addr);
    write_offset = write_start_addr - section_start_addr;
    if (section_size > sizeof(write_tmp))
        return false;

    while(true)
    {
        /* circumstances 1: store data size less than flash sector size and only none multiple sector write is needed */
        /* circumstances 2: store data size less than flash sector length but need to write from the end of the sector N to the start of the sector N + 1 */
        /* circumstances 3: store data size large than flash sector length */
        /* read whole section */
        if (Dev.read(p_dev->obj, section_start_addr, write_tmp, section_size))
            return false;

        /* erase whole section */
        if (Dev.erase(p_dev->obj, section_start_addr))
            return false;

        /* update whole section */
        if (write_offset + write_len > section_size)
            write_len = section_size - write_offset;
        
        /* copy data to section data read out */
        memcpy(write_tmp + write_offset, p_data, write_len);

        uint8_t state = Dev.write(p_dev->obj, section_start_addr, write_tmp, section_size);

        /* clear cache buff */
        memset(write_tmp, 0, section_size);
        
        len -= write_len;
        if (state == 0)
        {
            if (len == 0)
                return true;
        }
        else
            return false;

        write_offset = 0;
        next_write_addr = Dev.get_start_addr(p_dev->obj, section_start_addr + write_len);
        if (next_write_addr == section_start_addr)
            write_offset = write_len;

        p_data += write_len;
        write_len = len;
        section_start_addr = next_write_addr; 
    }

    return false;
}

static bool Storage_Dev_Param_Erase(StorageDevObj_TypeDef *p_dev, uint32_t base_addr, uint32_t len)
{
    uint32_t erase_start_addr = base_addr;

    if ((p_dev == NULL) || \
        (Dev.info == NULL) || \
        (Dev.erase == NULL) || \
        (p_dev->obj == NULL) || \
        (len == 0))
        return false;

    /* erase external flash sector */
    /* get w25qxx device info */
    /* address check */
    if (erase_start_addr < Dev.info(p_dev->obj).start_addr)
        return false;

    /* W25Qxx device read */
    if (Dev.erase(p_dev->obj, erase_start_addr))
        return false;
    
    return true;
}
