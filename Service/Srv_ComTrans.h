#ifndef __SRV_COMTRANS_H
#define __SRV_COMTRANS_H

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "cmsis_os.h"
#include "../common/util.h"

typedef struct
{
    bool init_state;
    void *port_obj;
    uint32_t rec_cnt;
    uint32_t rec_size;
    osSemaphoreId Tx_Irq_Sem;

    bool rx_reading;

    uint8_t *p_buf_1;
    uint16_t buf_1_size;

    uint8_t *p_buf_2;
    uint16_t buf_2_size;
} SrvComObj_TypeDef;

typedef struct
{
    bool (*init)(SrvComObj_TypeDef *obj);
    bool (*de_init)(SrvComObj_TypeDef *obj);
    bool (*write)(SrvComObj_TypeDef *obj, uint8_t *p_data, uint16_t len);
    uint16_t (*read)(SrvComObj_TypeDef *obj, uint8_t *p_data, uint16_t len);
    uint16_t (*available)(SrvComObj_TypeDef *obj);
} SrvComTrans_TypeDef;

extern SrvComTrans_TypeDef SrvCom;

#endif
