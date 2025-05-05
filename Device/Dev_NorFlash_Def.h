#ifndef __DEV_NORFLASH_DEF_H
#define __DEV_NORFLASH_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

typedef struct
{
    uint32_t prod_code;

    uint32_t start_addr;
    uint32_t flash_size;

    uint32_t block_size;
    uint32_t sector_size;
    uint16_t page_size;

    uint16_t block_num;
    uint16_t sector_num;
    uint16_t page_num;
} DevNorFlash_Info_TypeDef;

#ifdef __cplusplus
}
#endif

#endif
