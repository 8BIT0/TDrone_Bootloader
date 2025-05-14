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
static YModem_Handle YModem_Obj_Init(void *port_obj, YModem_TransDir_TypeDef dir, malloc_callback malloc_cb, free_callback free_cb, \
                                     trans_callback trans_cb, rec_start_callback rec_start_cb, rec_eot_callback rec_eot_cb,\
                                     rec_done_callback rec_done_cb, rec_pack_callback rec_pck_cb);
static void YModem_Rx(YModem_Handle YM_hdl, uint32_t t_update, uint32_t t_sys, uint8_t *buf, uint32_t size);
static void YModem_Tx(YModem_Handle YM_hdl, uint32_t sys_time, uint8_t *p_file, uint8_t *p_file_name, uint32_t file_size, uint8_t *p_rx_data, uint8_t rx_size);

/* internal function */
static int8_t YModem_Rx_Pack_Check(YModemObj_TypeDef *Obj, uint8_t *buf, uint32_t size);
static bool YModem_Rx_Check_Pack_Empty(uint8_t *buf, uint32_t size);
static void YModem_Obj_DeInit(YModem_Handle YM_hdl);
static void YModem_SendByte(YModemObj_TypeDef *obj, uint8_t byte);
static void YModem_Idle_Proc(YModemObj_TypeDef *Obj, uint8_t *buf, uint32_t size);
static void YModem_Ack_Proc(YModemObj_TypeDef *Obj, uint8_t *buf, uint32_t size);
static void YModem_EOT_Proc(YModemObj_TypeDef *Obj, uint8_t *buf, uint32_t size);
static void YModem_Check_Rx_Exit(YModemObj_TypeDef *Obj);

static bool YModem_Pack_File(YModemObj_TypeDef *Obj, uint8_t pck_header, uint8_t *p_buf, uint32_t buf_size);
static bool YModem_Pack_FileInfo(YModemObj_TypeDef *Obj, uint8_t pck_header, uint8_t *file_name, uint32_t file_size);

/* external vairable */
YModem_TypeDef YModem = {
    .Init = YModem_Obj_Init,
    .Rx   = YModem_Rx,
    .Tx   = YModem_Tx,
};

/* get pack type */
static int8_t YModem_Rx_Pack_Check(YModemObj_TypeDef *obj, uint8_t *buf, uint32_t size)
{
    uint16_t pck_crc = 0;
    uint16_t chk_crc = 0;

    if ((obj == NULL) || (buf == NULL) || (size == 0))
        return (int8_t)YModem_Rx_Pack_Error;
    
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
        return (int8_t)YModem_Rx_Pack_Error;

    if ((size >= 3) && (buf[1] + buf[2] != 0xff))
    {
        return (int8_t)YModem_Rx_Pack_Error;
    }

    if (size < (obj->pck_size + YMODEM_FUNC_BYTE_SIZE))
    {
        if (obj->t_update && ((obj->t_sys - obj->t_update) >= YMODEM_REC_TIMEOUT))
            return (int8_t)YModem_Rx_TimeOut;

        return (int8_t)YModem_Rx_Pack_Incomplete;
    }

    if (obj->pck_size)
    {
        chk_crc = Common_CRC16((buf + PACKET_HEADER), (obj->pck_size + YMODEM_FUNC_BYTE_SIZE - PACKET_OVERHEAD));
        pck_crc = (buf[size - 2] << 8) + buf[size - 1];

        if ((chk_crc == pck_crc) && (0xff == (buf[1] + buf[2])))
            return buf[0];
        
        return (int8_t)YModem_Rx_Pack_CRC_Error;
    }
    
    return (int8_t)YModem_Rx_Pack_Error;
}

static bool YModem_Rx_Check_Pack_Empty(uint8_t *buf, uint32_t size)
{
    uint8_t chk = 0;
    
    for (uint32_t i = 0; i < size; i++)
        chk |= buf[i];
    
    return (chk == 0) ? true : false;
}

static YModem_Handle YModem_Obj_Init(void *port_obj, YModem_TransDir_TypeDef dir, malloc_callback malloc_cb, free_callback free_cb, \
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
    obj->dir = dir;
    obj->port_obj = port_obj;
    obj->malloc_cb = malloc_cb;
    obj->free_cb = free_cb;
    obj->trans_cb = trans_cb;

    if (obj->dir == YModem_Dir_Tx)
    {
        /* create temporary transmit buff */
        obj->p_tx_tmp = malloc_cb(YMODEM_DATA_SIZE_1024 + YMODEM_FUNC_BYTE_SIZE);
        if (obj->p_tx_tmp == NULL)
        {
            free_cb(obj);
            return 0;
        }
    }
    else if (obj->dir == YModem_Dir_Rx)
    {
        obj->start_cb = rec_start_cb;
        obj->done_cb = rec_done_cb;
        obj->rec_pck_cb = rec_pck_cb;
        obj->rec_eot_cb = rec_eot_cb;

        obj->pck_cnt = 0;
        obj->pck_size = 0;
        obj->rx_status = YMODEM_RX_IDLE;
    }

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
            YModem_SendByte(Obj, ACK);
            if (!YModem_Rx_Check_Pack_Empty((buf + PACKET_HEADER), Obj->pck_size))
            {
                YModem_SendByte(Obj, CNC);
                if (Obj->pck_size)
                {
                    /* get file name and size from first pack */
                    if (Obj->start_cb)
                        Obj->start_cb(Obj->port_obj, buf, (Obj->pck_size + YMODEM_FUNC_BYTE_SIZE), &buf[PACKET_HEADER], Obj->pck_size, true);
                    

                    Obj->rx_status = YMODEM_RX_ACK;
                    return;
                }
                
                Obj->rx_status = YMODEM_RX_ERR;
            }
            break;

        case EOT: Obj->rx_status = YMODEM_RX_EXIT; break;
        case YModem_Rx_Pack_Incomplete: break;
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
        case YModem_Rx_Pack_Incomplete: break;
        case YModem_Rx_TimeOut:
        case YModem_Rx_Pack_CRC_Error:
        case YModem_Rx_Pack_Error:
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
            YModem_SendByte(Obj, ACK);
            YModem_SendByte(Obj, CNC);
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

static void YModem_Check_Rx_Exit(YModemObj_TypeDef *Obj)
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
            if (Obj->t_update && ((t_sys - Obj->t_update) >= YMODEM_TRANS_TIMEOUT))
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

    YModem_Check_Rx_Exit(Obj);
}

/*************************************************** transmit section untested ************************************************/
/******************************************************** transmit section ****************************************************/
static bool YModem_Pack_File(YModemObj_TypeDef *Obj, uint8_t pck_header, uint8_t *p_buf, uint32_t buf_size)
{
    uint16_t crc = 0;

    if ((Obj == NULL) || \
        (Obj->dir == YModem_Dir_Rx) || \
        (Obj->p_tx_tmp == NULL) || \
        (p_buf == 0) || (buf_size == 0))
        return false;

    Obj->tx_trans_size = 0;
    switch (pck_header)
    {
        case SOH: Obj->tx_trans_size = YMODEM_DATA_SIZE_128;  break;
        case STX: Obj->tx_trans_size = YMODEM_DATA_SIZE_1024; break;
        default: return false;
    }

    memset(Obj->p_tx_tmp, 0, Obj->tx_trans_size + YMODEM_FUNC_BYTE_SIZE);

    Obj->p_tx_tmp[0] = pck_header;
    Obj->p_tx_tmp[1] = Obj->tx_cyc;
    Obj->p_tx_tmp[2] = UINT8_MAX - Obj->tx_cyc;

    /* copy buff */
    if (buf_size >= Obj->tx_trans_size)
    {
        memcpy(Obj->p_tx_tmp + PACKET_HEADER, p_buf, Obj->tx_trans_size);
    }
    else
    {
        memcpy(Obj->p_tx_tmp + PACKET_HEADER, p_buf, buf_size);
    }

    crc = Common_CRC16(p_buf, Obj->tx_trans_size);
    Obj->p_tx_tmp[PACKET_HEADER + Obj->tx_trans_size]     = ((uint8_t*)(&crc))[0];
    Obj->p_tx_tmp[PACKET_HEADER + Obj->tx_trans_size + 1] = ((uint8_t*)(&crc))[1];
    Obj->tx_trans_size += YMODEM_FUNC_BYTE_SIZE;
    Obj->tx_cyc ++;

    /* transmit data */
    if (Obj->trans_cb)
    {
        Obj->trans_cb(Obj->port_obj, Obj->p_tx_tmp, Obj->tx_trans_size);
        return true;
    }
    
    return false;
}

static bool YModem_Pack_FileInfo(YModemObj_TypeDef *Obj, uint8_t pck_header, uint8_t *file_name, uint32_t file_size)
{
    char size_str[32];
    uint16_t crc = 0;

    if ((Obj == NULL) || (Obj->p_tx_tmp == NULL) || (file_name == NULL) || (file_size == 0))
        return false;

    Obj->tx_trans_size = 0;
    switch (pck_header)
    {
        case SOH: Obj->tx_trans_size = YMODEM_DATA_SIZE_128;  break;
        case STX: Obj->tx_trans_size = YMODEM_DATA_SIZE_1024; break;
        default: return false;
    }

    memset(size_str, '\0', sizeof(size_str));
    memset(Obj->p_tx_tmp, 0, YMODEM_DATA_SIZE_1024 + YMODEM_FUNC_BYTE_SIZE);
    Obj->tx_cyc = 0;

    Obj->p_tx_tmp[0] = pck_header;
    Obj->p_tx_tmp[1] = Obj->tx_cyc;
    Obj->p_tx_tmp[2] = UINT8_MAX - Obj->tx_cyc;

    snprintf(size_str, sizeof(size_str), "%ld", file_size);
    strcpy((char *)&Obj->p_tx_tmp[PACKET_HEADER], (char *)file_name);
    strcpy((char *)&Obj->p_tx_tmp[PACKET_HEADER + strlen((char *)file_name) + 1], size_str);

    crc = Common_CRC16(Obj->p_tx_tmp, Obj->tx_trans_size);
    Obj->p_tx_tmp[PACKET_HEADER + Obj->tx_trans_size]     = ((uint8_t*)(&crc))[0];
    Obj->p_tx_tmp[PACKET_HEADER + Obj->tx_trans_size + 1] = ((uint8_t*)(&crc))[1];
    Obj->tx_trans_size += YMODEM_FUNC_BYTE_SIZE;
    
    /* transmit data */
    if (Obj->trans_cb)
    {
        Obj->trans_cb(Obj->port_obj, Obj->p_tx_tmp, Obj->tx_trans_size);
        return true;
    }

    return false;
}

/* when ymodem under tx idle status and received letter 'C' from the receiver, transmit the first pack with file name and file size to receiver */
static void YModem_Tx_Idle_Proc(YModemObj_TypeDef *Obj, uint32_t sys_t, uint8_t *p_file_name, uint32_t file_size, uint8_t *p_rx_data, uint8_t rx_size)
{
    if ((Obj == NULL) || \
        (p_file_name == NULL) || \
        (file_size == 0))
        return;

    /* 20s timeout checking */
    if (Obj->t_update != 0)
    {
        /* keep 20s received nothing request from receiver */
        if ((sys_t - Obj->t_update >= YMODEM_TRANS_TIMEOUT * 10) && ((p_rx_data == NULL) || (rx_size == 0)))
        {
            Obj->tx_status = YMODEM_TX_ERR;
            return;
        }
    }
    else
        Obj->t_update = sys_t;

    /* check the newest request from the receiver */
    switch (p_rx_data[rx_size - 1])
    {
        case CNC:
            /* request first pack (1024 length) */
            if (!YModem_Pack_FileInfo(Obj, STX, p_file_name, file_size))
            {
                Obj->tx_status = YMODEM_TX_ERR;
                return;
            }

            Obj->t_update = sys_t;
            Obj->tx_status = YMODEM_TX_IDLE_ACK;
            if (Obj->start_cb)
                Obj->start_cb(Obj->port_obj, NULL, 0, NULL, 0, true);
            break;

        /* cnacel transmit */
        case CAN: Obj->tx_status = YMODEM_TX_EXIT; break;
        default: break;
    }
}

/* trans byte size 1024 */
static void YModem_Tx_DataPack_Proc(YModemObj_TypeDef *Obj, uint32_t sys_t, uint8_t *p_file, uint32_t file_size, uint8_t *p_rx_data, uint8_t rx_size)
{
    bool proto_pack = true;
    uint8_t *p_tmp = NULL;
    uint32_t remain_size = 0;

    if ((Obj == NULL) || (p_file == NULL) || (file_size == 0))
        return;

    if (Obj->tx_file_pack_index != 0)
    {
        /* wait ACK from receiver */
        if ((p_rx_data == NULL) || (rx_size == 0))
        {
            /* check request timeout */
            if (sys_t - Obj->t_update >= YMODEM_TRANS_TIMEOUT)
                Obj->tx_status = YMODEM_TX_ERR;
            return;
        }

        switch (p_rx_data[rx_size - 1])
        {
            case ACK: Obj->t_update = sys_t; break;
            case CAN:
                Obj->tx_status = YMODEM_TX_EXIT;
                proto_pack = false;
                break;
            default: return;
        }
    }

    if (proto_pack)
    {
        p_tmp = p_file + Obj->tx_file_pack_index * YMODEM_DATA_SIZE_1024;
        remain_size = Obj->tx_file_pack_num * YMODEM_DATA_SIZE_1024 - Obj->tx_file_pack_index * YMODEM_DATA_SIZE_1024;
        YModem_Pack_File(Obj, STX, p_tmp, remain_size);
        Obj->tx_file_pack_index ++;

        if (Obj->tx_file_pack_index == Obj->tx_file_pack_num)
        {
            Obj->tx_status = YMODEM_TX_EOT;
            Obj->eot_cnt = 0;
        }
    }
}

static void YModme_Tx_EOT_Proc(YModemObj_TypeDef *Obj, uint32_t t_sys, uint8_t *p_rx_data, uint8_t rx_size)
{
    uint8_t data = EOT;

    if (Obj->eot_cnt != 0)
    {
        /* wait NAK */
        if ((p_rx_data == NULL) || (rx_size != 1) || (Obj->eot_cnt > 1))
        {
            /* check NAK receive timeout */
            if (Obj->t_update && ((t_sys - Obj->t_update) >= YMODEM_TRANS_TIMEOUT))
                Obj->rx_status = YMODEM_TX_EXIT;
            return;
        }

        Obj->t_update = t_sys;
        if ((p_rx_data[0] == NAK) || (p_rx_data[0] == CAN))
        {
            Obj->tx_status = YMODEM_TX_EXIT;
            if (p_rx_data[0] == CAN)
                return;
        }
    }
    else
        Obj->t_update = t_sys;

    /* transmot EOT */
    if (Obj->trans_cb)
        Obj->trans_cb(Obj->port_obj, &data, 1);
    
    Obj->eot_cnt ++;
}

static void YModem_Tx_IdleACK_Proc(YModemObj_TypeDef *Obj, uint32_t sys_t, uint32_t file_size, uint8_t *p_rx_data, uint8_t rx_size)
{
    if ((Obj == NULL) || (p_rx_data == NULL) || (rx_size == 0))
        return;

    /* check the newest request from receiver */
    if (rx_size == 1)
    {
        switch (p_rx_data[0])
        {
            /* cancel transmit */
            case CAN: Obj->tx_status = YMODEM_TX_EXIT; break;
            case NAK:
                /* receiver request file info again */
                /* reset update time */
                Obj->tx_status = YMODEM_TX_IDLE;
                Obj->t_update = 0;
                break;
            default: break;
        }
    }
    else if ((rx_size == 2) && (p_rx_data[0] == ACK) && (p_rx_data[1] == CNC))
    {
        /* received ack from receiver */
        /* transmit data pack to receiver */
        Obj->tx_status = YMODEM_TX_DATA;
        Obj->t_update = Obj->t_sys;
        Obj->tx_file_pack_index = 0;
        Obj->tx_file_pack_num = file_size / YMODEM_DATA_SIZE_1024;
        if (file_size % YMODEM_DATA_SIZE_1024)
            Obj->tx_file_pack_num ++;
    }
    else if (sys_t - Obj->t_update >= YMODEM_TRANS_TIMEOUT)
    {
        /* unknown reply from the receiver */
        /* or got nothing reply from receiver */
        /* check for timeout */
        Obj->tx_status = YMODEM_TX_ERR;
    }
}

static void YModem_Check_Tx_Exit(YModemObj_TypeDef *Obj)
{
    uint8_t data = CAN;

    switch (Obj->tx_status)
    {
        case YMODEM_TX_ERR:
            if (Obj->trans_cb)
                Obj->trans_cb(Obj->port_obj, &data, 1);

            if (Obj->done_cb)
                Obj->done_cb(Obj->port_obj, YModem_Tx_Error);

        case YMODEM_TX_EXIT:
            Obj->t_sys = 0;
            Obj->tx_cyc = 0;
            Obj->eot_cnt = 0;
            Obj->t_update = 0;
            Obj->tx_trans_size = 0;
            Obj->tx_file_pack_num = 0;
            Obj->tx_file_pack_index = 0;
            Obj->tx_status = YMODEM_TX_IDLE;

            /* clear temporary buff */
            if (Obj->p_tx_tmp)
            {
                memset(Obj->p_tx_tmp, 0, YMODEM_DATA_SIZE_1024 + YMODEM_FUNC_BYTE_SIZE);
                Obj->free_cb(Obj->p_tx_tmp);
                Obj->p_tx_tmp = NULL;
            }

            if (Obj->done_cb)
                Obj->done_cb(Obj->port_obj, YModem_Tx_Done);
            break;

        default: break;
    }
}

static void YModem_Tx(YModem_Handle YM_hdl, uint32_t sys_time, uint8_t *p_file, uint8_t *p_file_name, uint32_t file_size, uint8_t *p_rx_data, uint8_t rx_size)
{
    YModemObj_TypeDef *Obj = To_YModem_Obj(YM_hdl);

    if ((Obj == NULL) || (Obj->dir == YModem_Dir_Rx) || (p_file == NULL) || (p_file_name == NULL) || (file_size == 0))
        return;
    
    switch (Obj->tx_status)
    {
        case YMODEM_TX_IDLE: YModem_Tx_Idle_Proc(Obj, sys_time, p_file_name, file_size, p_rx_data, rx_size); break;
        case YMODEM_TX_IDLE_ACK: YModem_Tx_IdleACK_Proc(Obj, sys_time, file_size, p_rx_data, rx_size); break;
        case YMODEM_TX_DATA: YModem_Tx_DataPack_Proc(Obj, sys_time, p_file, file_size, p_rx_data, rx_size); break;
        case YMODEM_TX_EOT: YModme_Tx_EOT_Proc(Obj, sys_time, p_rx_data, rx_size); break;
        default: break;
    }
    
    YModem_Check_Tx_Exit(Obj);
}
