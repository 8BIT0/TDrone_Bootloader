#ifndef __YMODEM_H
#define __YMODEM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

typedef uint32_t YModem_Handle;

typedef void (*trans_callback)(void *port_arg, uint8_t *p_data, uint16_t size);
typedef void (*rec_start_callback)(void *arg, uint8_t *p_data, uint16_t size, uint8_t *p_payload, uint16_t payload_size, bool valid);
typedef void (*rec_pack_callback)(void *arg, uint8_t *p_data, uint16_t size, uint8_t *p_payload, uint16_t payload_size, bool valid);
typedef void (*rec_eot_callback)(void *arg, uint8_t *p_data, uint16_t size);
typedef void (*rec_done_callback)(void *arg, int8_t code);
typedef void* (*malloc_callback)(uint32_t size);
typedef void (*free_callback)(void *ptr);

typedef enum
{
    YModem_Dir_Rx = 0,
    YModem_Dir_Tx,
} YModem_TransDir_TypeDef;

typedef enum
{
    YModem_Rx_Error_None      = 0,
    YModem_Rx_Done            = 1,
    YModem_Rx_Pack_Error      = -1,
    YModem_Rx_Pack_CRC_Error  = -2,
    YModem_Rx_Error           = -3,
    YModem_Rx_Pack_Incomplete = -4,
    YModem_Rx_TimeOut         = -5,
} YModem_ErrorCode_TypeDef;

typedef struct
{
    YModem_TransDir_TypeDef dir;

    uint16_t pck_cnt;
    uint16_t pck_size;
    uint8_t rx_status;
    void *port_obj;
    void *cus_obj;

    uint32_t t_sys;
    uint32_t t_update;

    /* valid when ymodem tx mode */
    uint16_t tx_trans_size;
    uint16_t tx_file_pack_index;
    uint16_t tx_file_pack_num;
    uint8_t tx_status;
    uint8_t tx_cyc;
    uint8_t *p_tx_tmp;

    malloc_callback malloc_cb;
    free_callback free_cb;
    trans_callback trans_cb;
    rec_start_callback start_cb;
    rec_pack_callback rec_pck_cb;
    rec_eot_callback rec_eot_cb;
    rec_done_callback done_cb;
} YModemObj_TypeDef;

typedef struct
{
    YModem_Handle (*Init)(void *port_obj, YModem_TransDir_TypeDef dir, malloc_callback malloc_cb, free_callback free_cb, \
                          trans_callback trans_cb, rec_start_callback rec_start_cb, rec_eot_callback rec_eot_cb,\
                          rec_done_callback rec_done_cb, rec_pack_callback rec_pck_cb);
    void (*Rx)(YModem_Handle YM_hdl, uint32_t t_update, uint32_t t_sys, uint8_t *buf, uint32_t size);
    void (*Tx)(YModem_Handle YM_hdl, uint32_t sys_time, uint8_t *p_file, uint8_t *p_file_name, uint32_t file_size, uint8_t *p_rx_data, uint16_t rx_size);
} YModem_TypeDef;

extern YModem_TypeDef YModem;

#ifdef __cplusplus
}
#endif

#endif
