#ifndef __SRV_COMTRANS_H
#define __SRV_COMTRANS_H

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "cmsis_os.h"
#include "../common/util.h"
#include "CusQueue.h"

typedef struct
{
    bool init_state;
    void *port_obj;
    uint32_t rec_cnt;
    osSemaphoreId Tx_Irq_Sem;
    QueueObj_TypeDef rec_Q;
    bool queue_inuse;

    uint16_t tmp_buf_remain;
    uint8_t *tmp_buf;
} SrvComObj_TypeDef;

typedef struct
{
    bool (*init)(SrvComObj_TypeDef *obj);
    bool (*de_init)(SrvComObj_TypeDef *obj);
    bool (*write)(SrvComObj_TypeDef *obj, uint8_t *p_data, uint16_t len);
    uint16_t (*read)(SrvComObj_TypeDef *obj, uint8_t *p_data, uint16_t len);
    bool (*available)(SrvComObj_TypeDef obj);
} SrvComTrans_TypeDef;

extern SrvComTrans_TypeDef SrvCom;

#endif
