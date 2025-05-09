#include "YModem.h"
#include "../util.h"

#define To_YModem_Obj(x)        ((YModemObj_TypeDef *)x)

#define YMODEM_FUNC_BYTE_SIZE   5

#define YMODEM_DATA_SIZE_128    128
#define YMODEM_DATA_SIZE_1024   1024
   
#define YMODEM_RX_IDLE          0
#define YMODEM_RX_ACK           1
#define YMODEM_RX_EOT           2
#define YMODEM_RX_ERR           3
#define YMODEM_RX_EXIT          4
#define YMODEM_RX_DONE          5

#define YMODEM_TX_IDLE          0
#define YMODEM_TX_IDLE_ACK      1
#define YMODEM_TX_DATA          2
#define YMODEM_TX_DATA_ACK      3
#define YMODEM_TX_EOT           4
#define YMODEM_TX_ERR           5
#define YMODEM_TX_EXIT          6

#define PACKET_SEQNO_INDEX      (1)  
#define PACKET_SEQNO_COMP_INDEX (2)  
  
#define PACKET_HEADER           (3)                 /* start, block, block-complement */  
#define PACKET_TRAILER          (2)                 /* CRC bytes */  
#define PACKET_OVERHEAD         (PACKET_HEADER + PACKET_TRAILER)  

/* ASCII control codes: */  
#define SOH                     ((uint8_t)0x01)     /* start of 128-byte data packet */  
#define STX                     ((uint8_t)0x02)     /* start of 1024-byte data packet */  
#define EOT                     ((uint8_t)0x04)     /* end of transmission */  
#define ACK                     ((uint8_t)0x06)     /* receive OK */  
#define NAK                     ((uint8_t)0x15)     /* receiver error; retry */  
#define CAN                     ((uint8_t)0x18)     /* two of these in succession abortas transfer */  
#define CNC                     ((uint8_t)0x43)     /* character CNC */
                                                      
#define YMODEM_REC_TIMEOUT      500                 /* if single package receive timeout over 500ms request again / timeout unit: ms */
#define YMODEM_TRANS_TIMEOUT    2000                /* if YModem keep 2s without any data received after the first package than abort transmit / timeout unit: ms */

/* external function */
static YModem_Handle YModem_Obj_Init(void *port_obj, malloc_callback malloc_cb, free_callback free_cb, \
                                     trans_callback trans_cb, rec_start_callback rec_start_cb, rec_eot_callback rec_eot_cb,\
                                     rec_done_callback rec_done_cb, rec_pack_callback rec_pck_cb);
static void YModem_Rx(YModem_Handle YM_hdl, uint32_t t_update, uint32_t t_sys, uint8_t *buf, uint32_t size);

/* internal function */
static int8_t YModem_Rx_Pack_Check(YModemObj_TypeDef *Obj, uint8_t *buf, uint32_t size);
static bool YModem_Rx_Check_Pack_Empty(uint8_t *buf, uint32_t size);
static void YModem_Obj_DeInit(YModem_Handle YM_hdl);
static void YModem_SendByte(YModemObj_TypeDef *obj, uint8_t byte);
static void YModem_Idle_Proc(YModemObj_TypeDef *Obj, uint8_t *buf, uint32_t size);
static void YModem_Ack_Proc(YModemObj_TypeDef *Obj, uint8_t *buf, uint32_t size);
static void YModem_EOT_Proc(YModemObj_TypeDef *Obj, uint8_t *buf, uint32_t size);
static void YModem_Check_Exit(YModemObj_TypeDef *Obj);

/* external vairable */
YModem_TypeDef YModem = {
    .Init = YModem_Obj_Init,
    .Rx   = YModem_Rx,
};

/* get pack type */
static int8_t YModem_Rx_Pack_Check(YModemObj_TypeDef *obj, uint8_t *buf, uint32_t size)
{
    uint16_t pck_crc = 0;
    uint16_t chk_crc = 0;

    if ((obj == NULL) || (buf == NULL) || (size == 0))
        return (int8_t)YModem_Pack_Error;
    
    obj->pck_size = 0;
    if ((buf[0] == SOH) || (buf[0] == STX))
    {
        obj->pck_size = (buf[0] == SOH) ? YMODEM_DATA_SIZE_128 : YMODEM_DATA_SIZE_1024;
    }
    else if (buf[0] == EOT)
    {
        obj->pck_size = 1;
        return EOT;
    }
    else
        return (int8_t)YModem_Pack_Error;

    if ((size >= 3) && (buf[1] + buf[2] != 0xff))
    {
        return (int8_t)YModem_Pack_Error;
    }

    if (size < (obj->pck_size + YMODEM_FUNC_BYTE_SIZE))
    {
        if (obj->t_update && ((obj->t_sys - obj->t_update) >= YMODEM_REC_TIMEOUT))
            return (int8_t)YModem_Rx_TimeOut;

        return (int8_t)YModem_Pack_Incomplete;
    }

    if (obj->pck_size)
    {
        chk_crc = Common_CRC16((buf + PACKET_HEADER), (obj->pck_size + YMODEM_FUNC_BYTE_SIZE - PACKET_OVERHEAD));
        pck_crc = (buf[size - 2] << 8) + buf[size - 1];

        if ((chk_crc == pck_crc) && (0xff == (buf[1] + buf[2])))
            return buf[0];
        
        return (int8_t)YModem_Pack_CRC_Error;
    }
    
    return (int8_t)YModem_Pack_Error;
}

static bool YModem_Rx_Check_Pack_Empty(uint8_t *buf, uint32_t size)
{
    uint8_t chk = 0;
    
    for (uint32_t i = 0; i < size; i++)
        chk |= buf[i];
    
    return (chk == 0) ? true : false;
}

static YModem_Handle YModem_Obj_Init(void *port_obj, malloc_callback malloc_cb, free_callback free_cb, \
                                     trans_callback trans_cb, rec_start_callback rec_start_cb, rec_eot_callback rec_eot_cb,\
                                     rec_done_callback rec_done_cb, rec_pack_callback rec_pck_cb)
{
    YModemObj_TypeDef *obj = NULL;

    if ((malloc_cb == NULL) || (free_cb == NULL))
        return 0;

    obj = malloc_cb(sizeof(YModemObj_TypeDef));
    if (obj == NULL)
    {
        free_cb(obj);
        return 0;
    }

    memset(obj, 0, sizeof(YModemObj_TypeDef));
    obj->port_obj = port_obj;
    obj->malloc_cb = malloc_cb;
    obj->free_cb = free_cb;
    obj->trans_cb = trans_cb;
    obj->start_cb = rec_start_cb;
    obj->done_cb = rec_done_cb;
    obj->rec_pck_cb = rec_pck_cb;
    obj->rec_eot_cb = rec_eot_cb;

    obj->pck_cnt = 0;
    obj->pck_size = 0;
    obj->rx_status = YMODEM_RX_IDLE;

    return (YModem_Handle)obj;
}

static void YModem_Obj_DeInit(YModem_Handle YM_hdl)
{
    YModemObj_TypeDef *obj = To_YModem_Obj(YM_hdl);
    free_callback free_cb = NULL;

    if ((obj == NULL) || (obj->free_cb == NULL))
        return;

    free_cb = obj->free_cb;

    memset(obj, 0, sizeof(YModemObj_TypeDef));
    free_cb(obj);
}

static void YModem_SendByte(YModemObj_TypeDef *obj, uint8_t byte)
{
    volatile uint8_t t_data = byte;

    if ((obj == NULL) || (obj->trans_cb == NULL) || (obj->port_obj == NULL))
        return;

    obj->trans_cb(obj->port_obj, (uint8_t *)&t_data, 1);
}

/******************************************************** receive section ****************************************************/
static void YModem_Idle_Proc(YModemObj_TypeDef *Obj, uint8_t *buf, uint32_t size)
{
    switch (YModem_Rx_Pack_Check(Obj, buf, size))
    {
        case SOH:
        case STX:
            if (YModem_Rx_Check_Pack_Empty((buf + PACKET_HEADER), Obj->pck_size))
            {
                YModem_SendByte(Obj, ACK);
                // Obj->rx_status = YMODEM_RX_EXIT;
                return;
            }
            else
            {
                if (Obj->pck_size)
                {
                    /* get file name and size from first pack */
                    if (Obj->start_cb)
                        Obj->start_cb(Obj->port_obj, buf, (Obj->pck_size + YMODEM_FUNC_BYTE_SIZE), &buf[PACKET_HEADER], Obj->pck_size, true);
                    
                    YModem_SendByte(Obj, ACK);
                    YModem_SendByte(Obj, CNC);

                    Obj->rx_status = YMODEM_RX_ACK;
                    return;
                }
                
                Obj->rx_status = YMODEM_RX_ERR;
            }
            break;

        case EOT: Obj->rx_status = YMODEM_RX_EXIT; break;
        case YModem_Pack_Incomplete: break;
        case YModem_Rx_TimeOut:
            YModem_SendByte(Obj, CAN);
            Obj->rx_status = YMODEM_RX_IDLE;
            if (Obj->start_cb)
                Obj->start_cb(Obj->port_obj, buf, (Obj->pck_size + YMODEM_FUNC_BYTE_SIZE), &buf[PACKET_HEADER], Obj->pck_size, false);
            break;

        default: Obj->rx_status = YMODEM_RX_ERR; break;
    }
}

static void YModem_Ack_Proc(YModemObj_TypeDef *Obj, uint8_t *buf, uint32_t size)
{
    if (Obj == NULL)
        return;

    switch(YModem_Rx_Pack_Check(Obj, buf, size))
    {
        case SOH:
        case STX:
            YModem_SendByte(Obj, ACK);

            if (Obj->rec_pck_cb)
                Obj->rec_pck_cb(NULL, buf, (Obj->pck_size + YMODEM_FUNC_BYTE_SIZE), &buf[PACKET_HEADER], Obj->pck_size, true);
            break;

        case EOT:
            YModem_SendByte(Obj, NAK);
            Obj->rx_status = YMODEM_RX_EOT;
            break;

        case CAN: Obj->rx_status = YMODEM_RX_ERR; break;
        case YModem_Pack_Incomplete: break;
        case YModem_Rx_TimeOut:
        case YModem_Pack_CRC_Error:
        case YModem_Pack_Error:
        default:
            YModem_SendByte(Obj, NAK);
            if (Obj->rec_pck_cb)
                Obj->rec_pck_cb(NULL, buf, (Obj->pck_size + YMODEM_FUNC_BYTE_SIZE), &buf[PACKET_HEADER], Obj->pck_size, false);
            break;
    }
}

static void YModem_EOT_Proc(YModemObj_TypeDef *Obj, uint8_t *buf, uint32_t size)
{
    if (Obj == NULL)
        return;

    switch (YModem_Rx_Pack_Check(Obj, buf, size))
    {
        case EOT:
            YModem_SendByte(Obj, CNC);
            YModem_SendByte(Obj, ACK);
            Obj->rx_status = YMODEM_RX_DONE;
            if (Obj->rec_eot_cb)
                Obj->rec_eot_cb(NULL, buf, Obj->pck_size);
            break;

        default: Obj->rx_status = YMODEM_RX_ERR; break;
    }
}

static void YModem_RX_Done_Proc(YModemObj_TypeDef *Obj, uint8_t *buf, uint32_t size)
{
    if (Obj == NULL)
        return;

    if (YModem_Rx_Pack_Check(Obj, buf, size) > 0)
    {
        /* final pack carry empty data in payload section (all 0) */
        YModem_SendByte(Obj, ACK);
        Obj->rx_status = YMODEM_RX_EXIT;
    }
}

static void YModem_Check_Exit(YModemObj_TypeDef *Obj)
{
    if (Obj == NULL)
        return;

    YModem_SendByte(Obj, CAN);
    switch (Obj->rx_status)
    {
        case YMODEM_RX_ERR:
            if (Obj->done_cb)
                Obj->done_cb(Obj->port_obj, YModem_Rx_Error);
        
        case YMODEM_RX_EXIT:
            if (Obj->done_cb)
                Obj->done_cb(Obj->port_obj, YModem_Rx_Done);
            
            Obj->t_update = 0;
            Obj->t_sys = 0;

            YModem_Obj_DeInit((YModem_Handle)Obj);
            break;

        default: break;
    }
}

static void YModem_Rx(YModem_Handle YM_hdl, uint32_t t_update, uint32_t t_sys, uint8_t *buf, uint32_t size)
{
    YModemObj_TypeDef *Obj = To_YModem_Obj(YM_hdl);

    if (Obj == NULL)
        return;

    if (size == 0)
    {
        if (Obj->rx_status != YMODEM_RX_IDLE)
        {
            if (Obj->t_update && ((Obj->t_sys - Obj->t_update) >= YMODEM_TRANS_TIMEOUT))
                Obj->rx_status = YMODEM_RX_ERR;
        }
        else
            YModem_SendByte(Obj, CNC);

        return;
    }

    Obj->t_sys = t_sys;
    if (t_update)
        Obj->t_update = t_update;

    switch (Obj->rx_status)
    {
        case YMODEM_RX_IDLE: YModem_Idle_Proc(Obj, buf, size);    break;
        case YMODEM_RX_ACK:  YModem_Ack_Proc(Obj, buf, size);     break;
        case YMODEM_RX_EOT:  YModem_EOT_Proc(Obj, buf, size);     break;
        case YMODEM_RX_DONE: YModem_RX_Done_Proc(Obj, buf, size); break;
        default: Obj->rx_status = YMODEM_RX_ERR; break;
    }

    YModem_Check_Exit(Obj);
}
