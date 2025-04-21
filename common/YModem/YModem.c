#include "YModem.h"
#include "../util.h"

#define To_YModem_Obj(x)        ((YModemObj_TypeDef *)x)

#define YMODEM_DATA_SIZE_128    128
#define YMODEM_DATA_SIZE_1024   1024
   
#define YMODEM_RX_IDLE          0
#define YMODEM_RX_ACK           1
#define YMODEM_RX_EOT           2
#define YMODEM_RX_ERR           3
#define YMODEM_RX_EXIT          4

#define YMODEM_TX_IDLE          0
#define YMODEM_TX_IDLE_ACK      1
#define YMODEM_TX_DATA          2
#define YMODEM_TX_DATA_ACK      3
#define YMODEM_TX_EOT           4
#define YMODEM_TX_ERR           5
#define YMODEM_TX_EXIT          6

#define PACKET_SEQNO_INDEX      (1)  
#define PACKET_SEQNO_COMP_INDEX (2)  
  
#define PACKET_HEADER           (3)     /* start, block, block-complement */  
#define PACKET_TRAILER          (2)     /* CRC bytes */  
#define PACKET_OVERHEAD         (PACKET_HEADER + PACKET_TRAILER)  
#define PACKET_TIMEOUT          (1)  

#define YMODEM_PAC_EMPTY        2       //包校验正确，但是里面是空值，在（IDLE状态，判断是否需要结束，退出）

/* ASCII control codes: */  
#define SOH                     (0x01)      /* start of 128-byte data packet */  
#define STX                     (0x02)      /* start of 1024-byte data packet */  
#define EOT                     (0x04)      /* end of transmission */  
#define ACK                     (0x06)      /* receive OK */  
#define NAK                     (0x15)      /* receiver error; retry */  
#define CAN                     (0x18)      /* two of these in succession abortas transfer */  
#define CNC                     (0x43)      /* character 'C' */  

/* external function */
static YModem_Handle YModem_Obj_Init(void *port_obj, malloc_callback malloc_cb, free_callback free_cb, \
                                     trans_callback trans_cb, rec_start_callback rec_start_cb, \
                                     rec_done_callback rec_done_cb, rec_pack_callback rec_pck_cb);
static void YModem_Rx(YModem_Handle YM_hdl, uint8_t *buf, uint32_t size);

/* internal function */
static int8_t YModem_Rx_Pack_Check(uint8_t *buf, uint32_t size);
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
static int8_t YModem_Rx_Pack_Check(uint8_t *buf, uint32_t size)
{
    uint8_t ch = 0;
    uint32_t index = 1;
    uint16_t pck_crc = 0;
    uint16_t chk_crc = 0;

    if ((buf == NULL) || (size == 0))
        return YModem_Pack_Error;

    ch = buf[0];
    if (size < YMODEM_DATA_SIZE_128)
    {
        /* receive cmd pack */
        if ((ch == EOT) || (ch == ACK) || (ch == NAK) || (ch == CAN) || (ch == CNC))
        {
            while ((index < size) && (buf[index ++] == ch));
            if(size == index)
                return ch;
        }
        
        return YModem_CMD_Code_Error;
    }

    if ((ch == SOH) || (ch == STX))
    {
        chk_crc = Common_CRC16((buf + PACKET_HEADER), (size - PACKET_OVERHEAD));
        pck_crc = (buf[size - 2] << 8) + buf[size - 1];

        if ((chk_crc == pck_crc) && (0xff == (buf[1] + buf[2])))
            return ch;
        
        return YModem_Pack_CRC_Error;
    }
    
    return YModem_CMD_Code_Error;
}

static bool YModem_Rx_Check_Pack_Empty(uint8_t *buf, uint32_t size)
{
    uint8_t chk = 0;
    
    for (uint32_t i = 0; i < size; i++)
        chk |= buf[i];
    
    if (chk == 0)
        return true;
    
    return false;
}

static YModem_Handle YModem_Obj_Init(void *port_obj, malloc_callback malloc_cb, free_callback free_cb, trans_callback trans_cb, rec_start_callback rec_start_cb, rec_done_callback rec_done_cb, rec_pack_callback rec_pck_cb)
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
    uint8_t t_data = byte;

    if ((obj == NULL) || (obj->trans_cb == NULL) || (obj->port_obj == NULL))
        return;

    obj->trans_cb(obj->port_obj, &t_data, 1);
}

/******************************************************** receive section ****************************************************/
static void YModem_Idle_Proc(YModemObj_TypeDef *Obj, uint8_t *buf, uint32_t size)
{
    switch (YModem_Rx_Pack_Check(buf, size))
    {
        case SOH:
        case STX:
            Obj->pck_size = ((buf[0] == SOH) ? YMODEM_DATA_SIZE_128 : YMODEM_DATA_SIZE_1024);
                
            if (YModem_Rx_Check_Pack_Empty((buf + PACKET_HEADER), Obj->pck_size))
            {
                YModem_SendByte(Obj, ACK);
                Obj->rx_status = YMODEM_RX_EXIT;
                return;
            }
            else
            {
                if (Obj->pck_size == YMODEM_DATA_SIZE_128)
                {
                    /* get file name and size from first pack */
                    if (Obj->start_cb)
                        Obj->start_cb(Obj->port_obj, buf[PACKET_HEADER], Obj->pck_size);
                    
                    YModem_SendByte(Obj, ACK);
                    YModem_SendByte(Obj, 'C');
                    
                    Obj->rx_status = YMODEM_RX_ACK;
                    return;
                }
                
                Obj->rx_status = YMODEM_RX_ERR;
            }
            break;

        case EOT: Obj->rx_status = YMODEM_RX_EXIT; break;
        default:  Obj->rx_status = YMODEM_RX_ERR; break;
    }
}

static void YModem_Ack_Proc(YModemObj_TypeDef *Obj, uint8_t *buf, uint32_t size)
{
    if (Obj == NULL)
        return;

    switch(YModem_Rx_Pack_Check(buf, size))
    {
        case SOH:
        case STX:
            YModem_SendByte(Obj, ACK);
            Obj->pck_size = ((buf[0] == SOH) ? YMODEM_DATA_SIZE_128 : YMODEM_DATA_SIZE_1024);

            if (Obj->rec_pck_cb)
                Obj->rec_pck_cb(NULL, &buf[PACKET_HEADER], Obj->pck_size);

            YModem_SendByte(Obj, 'C');
        break;

        case EOT:
            YModem_SendByte(Obj, NAK);
            Obj->rx_status = YMODEM_RX_EOT;
            break;

        case CAN: Obj->rx_status = YMODEM_RX_ERR; break;
        default: YModem_SendByte(Obj, NAK); break;
    }
}

static void YModem_EOT_Proc(YModemObj_TypeDef *Obj, uint8_t *buf, uint32_t size)
{
    if (Obj == NULL)
        return;

    switch (YModem_Rx_Pack_Check(buf, size))
    {
        case EOT:
            YModem_SendByte(Obj, ACK);
            Obj->rx_status = YMODEM_RX_EXIT;
            break;

        default: Obj->rx_status = YMODEM_RX_ERR; break;
    }
}

static void YModem_Check_Exit(YModemObj_TypeDef *Obj)
{
    if (Obj == NULL)
        return;

    switch (Obj->rx_status)
    {
        case YMODEM_RX_ERR:
            YModem_SendByte(Obj, CAN);
            if (Obj->done_cb)
                Obj->done_cb(NULL, YModem_Rx_Error);
        
        case YMODEM_RX_EXIT:
            if (Obj->done_cb)
                Obj->done_cb(NULL, YModem_Rx_Done);
            YModem_Obj_DeInit((YModem_Handle)Obj);
            break;

        default: break;
    }
}

static void YModem_Rx(YModem_Handle YM_hdl, uint8_t *buf, uint32_t size)
{
    YModemObj_TypeDef *Obj = To_YModem_Obj(YM_hdl);

    if (size == 0)
    {
        YModem_SendByte(Obj, 'C');
        return;
    }
    
    switch (Obj->rx_status)
    {
        case YMODEM_RX_IDLE: YModem_Idle_Proc(Obj, buf, size); break;
        case YMODEM_RX_ACK:  YModem_Ack_Proc(Obj, buf, size);  break;
        case YMODEM_RX_EOT:  YModem_EOT_Proc(Obj, buf, size);  break;
        default: Obj->rx_status = YMODEM_RX_ERR; break;
    }

    YModem_Check_Exit(Obj);
}
