#ifndef __BSP_FLASH_PORT_DEF_H
#define __BSP_FLASH_PORT_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef struct
{
    bool (*init)(void);
    void (*de_init)(void);
    uint8_t (*get_align_size)(void);
    
    bool (*erase)(uint32_t addr, uint32_t len);
    bool (*read)(uint32_t addr, uint8_t *p_data, uint32_t size);
    bool (*write)(uint32_t addr, uint8_t *p_data, uint32_t size);

    uint32_t (*total_size)(void);
    uint32_t (*sector_size)(uint16_t sector_index);
} BspFlash_TypeDef;

#ifdef __cplusplus
}
#endif

#endif
