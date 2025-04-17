#include "Ymodem.h"
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
  
// static uint8_t ym_tx_status = YMODEM_RX_IDLE;
// static uint32_t ym_tx_fil_sz = 0;
// static uint8_t *ym_tx_pbuf = NULL;
// static uint8_t ym_cyc = 0;

/* external function */
static YModem_Handle YModem_Obj_Init(YModem_Trans_TypeDef type, void *port_obj, \
                                     malloc_callback malloc_cb, free_callback free_cb, \
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

static YModem_Handle YModem_Obj_Init(YModem_Trans_TypeDef type, void *port_obj, malloc_callback malloc_cb, free_callback free_cb, trans_callback trans_cb, rec_start_callback rec_start_cb, rec_done_callback rec_done_cb, rec_pack_callback rec_pck_cb)
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
    obj->type = type;
    obj->port_obj = port_obj;
    obj->malloc_cb = malloc_cb;
    obj->free_cb = free_cb;
    obj->trans_cb = trans_cb;
    obj->start_cb = rec_start_cb;
    obj->done_cb = rec_done_cb;
    obj->rec_pck_cb = rec_pck_cb;

    obj->pck_cnt = 0;
    obj->pck_size = 0;
    obj->seek = 0;
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
    if (Obj == NULL)
        return;

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
                    
                    Obj->seek = 0;
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
                Obj->rec_pck_cb(&buf[PACKET_HEADER], Obj->seek, Obj->pck_size);

            Obj->seek += Obj->pck_size;
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
            YModem_Obj_DeInit((YModem_Handle)Obj);
            if (Obj->done_cb)
                Obj->done_cb(NULL, YModem_Rx_Done);
            break;

        default: break;
    }
}

static void YModem_Rx(YModem_Handle YM_hdl, uint8_t *buf, uint32_t size)
{
    YModemObj_TypeDef *Obj = To_YModem_Obj(YM_hdl);

    if (Obj == NULL)
        return;

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















































// //**********************************************************************发送部分
// //pbuf 是指向缓冲区的最开始的地方， pac_sz 是数据区的大小
// uint8 ymodem_tx_make_pac_data( char *pbuf, size_t pac_sz )
// {
//   uint8 ans = YMODEM_ERR;
//   uint16 crc;
  
//   pbuf[0] = pac_sz==128? SOH:STX;
//   pbuf[1] = ym_cyc;
//   pbuf[2] = ~ym_cyc;
//   crc = crc16( (unsigned char const*)pbuf, pac_sz );
//   pbuf[PACKET_HEADER+pac_sz]   = (u8)(crc/256);
//   pbuf[PACKET_HEADER+pac_sz+1] = (u8)(crc&0x00ff);
//   ym_cyc++;
//   return ans;
// }
// uint8 ymodem_tx_make_pac_header( char *pbuf, char *fil_nm, size_t fil_sz )
// {
//   uint8 ans = YMODEM_ERR;
//   uint8 nm_len;
//   memset( pbuf+PACKET_HEADER, 0, 128);
//   if( fil_nm )
//   {
//     nm_len = strlen( fil_nm );
//     strcpy( pbuf+PACKET_HEADER, fil_nm );
//     strcpy( pbuf+PACKET_HEADER+nm_len+1, u32_to_str( fil_sz ) );
//   }
//   ym_cyc = 0x00;
//   ymodem_tx_make_pac_data( pbuf, 128 );
//   return ans;
// }
// /*********************************************************************
//  * @fn      ymodem_tx_put : Ymodem发送时，逻辑轮转调用函数
//  * @param   buf : 数据缓冲区 buf : 数据大小
//  * 说明：
//  * 1.发送 [包  头] 状态：如果没有文件名，则发送空包，否则发送封装的头包
//  * 2.发送 [数据包] 状态：发送数据包，出现问题或结束，则进入结束状态
//  * 3.发送 [结  束] 状态：处理发送完成的相关事情
//  */ 
// void ymodem_tx_put( char *buf, size_t rx_sz )
// {
//   char *fil_nm=NULL;
//   size_t fil_sz=NULL;
//   switch( ym_tx_status )
//   {
//   case YMODEM_TX_IDLE:
//     switch( YModem_Rx_Pack_Check( buf, rx_sz ) )   //检查当前包是否合法,并返回包的类型
//     {
//     case CNC:
//       {
//         if( NULL == ym_tx_pbuf )
//         {
//           ym_tx_pbuf = pvPortMalloc( PACKET_OVERHEAD + PACKET_1K_SIZE );
//           if( NULL == ym_tx_pbuf )      //申请失败，则返回
//             break;
//         }
//         if( YModem_Error_None == ymodem_tx_header( &fil_nm, &fil_sz ) )   //得到 文件名和大小
//         {//封装一个包头，然后发送出去
//           ym_tx_fil_sz = fil_sz;
//           ymodem_tx_make_pac_header( ym_tx_pbuf, fil_nm, fil_sz );
//           __putbuf( ym_tx_pbuf, PACKET_OVERHEAD+PACKET_SIZE );
//           ym_tx_status = YMODEM_TX_IDLE_ACK;
//         }
//         else //封装一个空包，然后发出去
//         {
//           ymodem_tx_make_pac_header( ym_tx_pbuf, NULL, NULL );
//           __putbuf( ym_tx_pbuf, PACKET_OVERHEAD+PACKET_SIZE );
//         }
//       }
//     break;
//     case CAN:
//       ym_rx_status = YMODEM_TX_ERR;
//       goto err_tx;
//     break;
//     default:
//      goto err_tx;              //这儿暂时认为，包有误，就退出
//      break;
//     }
//     break;
//     case YMODEM_TX_IDLE_ACK:
//       {
//         switch( YModem_Rx_Pack_Check( buf, rx_sz ) )   //检查当前包是否合法,并返回包的类型
//         {
//         case ACK://准备发数据包
//           ym_tx_status = YMODEM_TX_DATA;
//           break;
//         case NAK://准备重发包头
//           ym_tx_status = YMODEM_TX_IDLE;
//           break;
//         default://啥也不做
//           break;
//         }
//       }
//       break;
// dt_tx: case YMODEM_TX_DATA:                             //1级——文件发送状态中
//       switch( YModem_Rx_Pack_Check( buf, rx_sz ) )   //检查当前包是否合法,并返回包的类型
//       {
//         case CNC:
//           if( YModem_Error_None == ymodem_tx_pac_get( ym_tx_pbuf+PACKET_HEADER, seek, PACKET_1K_SIZE ) )  //读取下一组数据
//           {
//             if( YModem_Error_None == ymodem_tx_make_pac_data( ym_tx_pbuf, PACKET_1K_SIZE ) )
//             {
//               __putbuf( ym_tx_pbuf, PACKET_OVERHEAD+PACKET_1K_SIZE );
//               ym_tx_status = YMODEM_TX_DATA_ACK;
//             }
//             else        //读取数据出错，结束传输
//             {
//               ym_tx_status = YMODEM_TX_ERR;
//               goto err_tx;
//             }
//           }
//           break;
//         case CAN:
//           ym_rx_status = YMODEM_TX_ERR;
//           goto err_tx;
//           break;
//         default:        //暂时啥也不做
//           break;
//       }
//       break;
//     case YMODEM_TX_DATA_ACK:
//       {
//         switch( YModem_Rx_Pack_Check( buf, rx_sz ) )   //检查当前包是否合法,并返回包的类型
//         {
//         case ACK:
//           seek += PACKET_1K_SIZE;
//           if( seek < ym_tx_fil_sz )     //数据未发送完（不能加‘=’！）
//             ym_tx_status = YMODEM_TX_DATA_ACK;
//           else  //数据发送完
//           {
//             ym_tx_status = YMODEM_TX_EOT;
//             __putchar( EOT );
//           }
//         break;
//         case CNC:       //如果接收方不先应答[ACK]而是直接发'C'在这里处理
//           seek += PACKET_1K_SIZE;
//           if( seek < ym_tx_fil_sz )     //数据未发送完（不能加‘=’！）
//           {
//             ym_tx_status = YMODEM_TX_DATA_ACK;
//             //下面的状态，因为我需要马上回复，所以用goto
//             goto dt_tx;         //发送下一个数据包
//           }
//           else  //数据发送完
//           {
//             ym_tx_status = YMODEM_TX_EOT;
//             __putchar( EOT );
//           }
//         break;
//         default:
//         break;
//         }
//       }
//     break;
//     case YMODEM_TX_EOT:
//     {
//       switch( YModem_Rx_Pack_Check( buf, rx_sz ) )   //检查当前包是否合法,并返回包的类型
//       {
//           //指令包
//         case NAK:
//           __putchar( EOT );
//           break;
//         case ACK:
//           __putchar( ACK );
//           ymodem_tx_finish( YModem_Error_None );
//           ym_rx_status = YMODEM_TX_IDLE;
//           break;
//         default:
//           break;
//       }
//     }
//       break;
// err_tx:  case YMODEM_TX_ERR:         //在这里放弃保存文件,终止传输
//       __putchar( CAN );
//       ymodem_rx_finish( YMODEM_ERR );
//       //break;                    //没有break，和下面公用代码
//   case YMODEM_TX_EXIT:        //到这里，就收拾好，然后退出
//       ym_rx_status = YMODEM_RX_IDLE;
//       //*这里还需要进行某些操作，使在退出后，不会再重新进入ymodem_rx_put()函数
//       vPortFree( ym_tx_pbuf );
//       ym_tx_pbuf = NULL;
//       usart_protocol_model_cur = USART_PROTOCOL_DEFAULT;
//       return;
//     default:
//       break;
//   }
// }

