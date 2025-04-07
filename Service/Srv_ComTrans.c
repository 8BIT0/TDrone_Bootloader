#include "Srv_ComTrans.h"
#include "Srv_OsCommon.h"
#include "HW_Def.h"

#define SRV_COM_PORT            USART1
#define SRV_COM_BAUDRATE        921600
#define SRV_COM_TX_PIN          Uart1_TxPin
#define SRV_COM_RX_PIN          Uart1_RxPin
#define SRV_COM_TX_DMA          Bsp_DMA_2
#define SRV_COM_TX_DMA_STREAM   Bsp_DMA_Stream_0
#define SRV_COM_RX_DMA          Bsp_DMA_2
#define SRV_COM_RX_DMA_STREAM   Bsp_DMA_Stream_1
#define SRV_COM_TX_BUFF_LEN     (2 Kb)
#define SRV_COM_RX_BUFF_LEN     (2 Kb)
#define SRV_COM_QUEUE_SIZE      (2 Kb)
#define SRV_COM_TX_TIMEOUT      100

/* internal variable */
// static __attribute__((section(".Perph_Section"))) uint8_t SrvCom_Tx_Buff[SRV_COM_TX_BUFF_LEN] = {0};
static __attribute__((section(".Perph_Section"))) uint8_t SrvCom_Rx_Buff[SRV_COM_RX_BUFF_LEN] = {0};
static uint8_t SrvCom_Rx_tmpBuf[SRV_COM_RX_BUFF_LEN];

/* internal function */
static void SrvComTrans_Rx_Callback(uint32_t cust_data_addr, uint8_t *buff, uint16_t size);
static void SrvComTrans_Tx_Callback(uint32_t cust_data_addr, uint8_t *buff, uint16_t size);

/* external function */
static bool SrvComTrans_Init(SrvComObj_TypeDef *obj);
static bool SrvComTrans_DeInit(SrvComObj_TypeDef *obj);
static bool SrvComTrans_DataAvailable(SrvComObj_TypeDef obj);
static bool SrvComTrans_Write(SrvComObj_TypeDef *obj, uint8_t *p_data, uint16_t len);
static uint16_t SrvComTrans_GetRecData(SrvComObj_TypeDef *obj, uint8_t *p_data, uint16_t len);

/* external variable */
SrvComTrans_TypeDef SrvCom = {
    .init       = SrvComTrans_Init,
    .de_init    = SrvComTrans_DeInit,
    .write      = SrvComTrans_Write,
    .read       = SrvComTrans_GetRecData,
    .available  = SrvComTrans_DataAvailable,
};

static bool SrvComTrans_Init(SrvComObj_TypeDef *obj)
{
    bool state = true;

    if (obj == NULL)
        return false;

    obj->init_state = false;

    // memset(SrvCom_Tx_Buff,   0, SRV_COM_TX_BUFF_LEN);
    memset(SrvCom_Rx_Buff,   0, SRV_COM_RX_BUFF_LEN);
    memset(SrvCom_Rx_tmpBuf, 0, SRV_COM_RX_BUFF_LEN);

    // obj->p_tx_buff = SrvCom_Tx_Buff;

    /* create port object */
    obj->port_obj = SrvOsCommon.malloc(BspUartObj_Size);
    if (obj->port_obj == NULL)
        return false;
    
    To_BspUart_ObjPtr(obj->port_obj)->hdl = SrvOsCommon.malloc(UART_HandleType_Size);
    if (To_BspUart_ObjPtr(obj->port_obj)->hdl == NULL)
    {
        SrvOsCommon.free(obj->port_obj);
        return false;
    }

    To_BspUart_ObjPtr(obj->port_obj)->rx_dma_hdl = (DMA_HandleTypeDef *)SrvOsCommon.malloc(UART_DMA_Handle_Size);
    To_BspUart_ObjPtr(obj->port_obj)->tx_dma_hdl = (DMA_HandleTypeDef *)SrvOsCommon.malloc(UART_DMA_Handle_Size);

    To_BspUart_ObjPtr(obj->port_obj)->instance          = SRV_COM_PORT;
    To_BspUart_ObjPtr(obj->port_obj)->baudrate          = SRV_COM_BAUDRATE;
    To_BspUart_ObjPtr(obj->port_obj)->pin_swap          = false;
    To_BspUart_ObjPtr(obj->port_obj)->tx_io             = SRV_COM_TX_PIN;
    To_BspUart_ObjPtr(obj->port_obj)->rx_io             = SRV_COM_RX_PIN;
    To_BspUart_ObjPtr(obj->port_obj)->tx_dma            = SRV_COM_TX_DMA;
    To_BspUart_ObjPtr(obj->port_obj)->tx_stream         = SRV_COM_TX_DMA_STREAM;
    To_BspUart_ObjPtr(obj->port_obj)->rx_dma            = SRV_COM_RX_DMA;
    To_BspUart_ObjPtr(obj->port_obj)->rx_stream         = SRV_COM_RX_DMA_STREAM;
    To_BspUart_ObjPtr(obj->port_obj)->rx_size           = SRV_COM_RX_BUFF_LEN;
    To_BspUart_ObjPtr(obj->port_obj)->rx_buf            = SrvCom_Rx_Buff;
    To_BspUart_ObjPtr(obj->port_obj)->cust_data_addr    = (uint32_t)obj;

    /* init port object */
    /* create tx semaphore */
    state &= BspUart.init(To_BspUart_ObjPtr(obj->port_obj));
    state &= Queue.create_auto(&obj->rec_Q, "ComRecv", SRV_COM_QUEUE_SIZE);


    osSemaphoreDef(ComTx);
    obj->Tx_Irq_Sem = osSemaphoreCreate(osSemaphore(ComTx), 1);

    if ((obj->Tx_Irq_Sem == NULL) || !state)
    {
        BspUart.de_init(To_BspUart_ObjPtr(obj->port_obj));
        SrvOsCommon.free(To_BspUart_ObjPtr(obj->port_obj)->hdl);
        SrvOsCommon.free(obj->port_obj);
        return false;
    }

    /* set callback */
    To_BspUart_ObjPtr(obj->port_obj)->TxCallback = SrvComTrans_Tx_Callback;
    To_BspUart_ObjPtr(obj->port_obj)->RxCallback = SrvComTrans_Rx_Callback;

    obj->tmp_buf = SrvCom_Rx_tmpBuf;
    obj->tmp_buf_remain = sizeof(SrvCom_Rx_tmpBuf);

    obj->init_state = true;
    return true;
}

static bool SrvComTrans_DeInit(SrvComObj_TypeDef *obj)
{
    bool state = true;

    if ((obj == NULL) || (obj->init_state == false))
        return false;

    /* de-init com port */
    state = BspUart.de_init(To_BspUart_ObjPtr(obj->port_obj));

    /* free malloc */
    SrvOsCommon.free(To_BspUart_ObjPtr(obj->port_obj)->hdl);
    SrvOsCommon.free(obj->port_obj);

    return state;
}

static void SrvComTrans_Tx_Callback(uint32_t cust_data_addr, uint8_t *buff, uint16_t size)
{
    SrvComObj_TypeDef *p_obj = (SrvComObj_TypeDef *)cust_data_addr;
    
    if ((p_obj == NULL) || !p_obj->init_state)
        return;

    /* release Tx semaphore */
    if (p_obj->Tx_Irq_Sem)
        osSemaphoreRelease(p_obj->Tx_Irq_Sem);
}

static void SrvComTrans_Rx_Callback(uint32_t cust_data_addr, uint8_t *buff, uint16_t size)
{
    SrvComObj_TypeDef *p_obj = (SrvComObj_TypeDef *)cust_data_addr;
    uint16_t tmp_buf_size = SRV_COM_RX_BUFF_LEN - p_obj->tmp_buf_remain;
    Queue_state q_state = Queue_ok;
    uint16_t i = 0;

    if ((p_obj == NULL) || !p_obj->init_state || (buff == NULL) || (size == 0))
        return;

    p_obj->rec_cnt ++;
    q_state = Queue.state(p_obj->rec_Q);
    if (!p_obj->queue_inuse && (q_state != Queue_full))
    {
        /* push tmp buff data into queue */
        if (tmp_buf_size)
        {
            for (i = 0; i < tmp_buf_size; i++)
            {
                q_state = Queue.push(&p_obj->rec_Q, &p_obj->tmp_buf[i], 1);
                if (q_state != Queue_ok)
                {
                    /* queue is full stop push temporary buff data to queue */
                    memmove(p_obj->tmp_buf, p_obj->tmp_buf + i, tmp_buf_size - i);
                    break;
                }
            }
        }

        if ((q_state == Queue_ok) || (q_state == Queue_empty))
        {
            /* push buff data into queue */
            for (i = 0; i < size; i ++)
            {
                q_state = Queue.push(&p_obj->rec_Q, &buff[i], 1);
                if (q_state != Queue_ok)
                {
                    /* queue is full stop push data to queue */
                    buff += i;
                    size -= i;

                    /* update temporary buff */
                    i = 0;
                    break;
                }
            }

            /* push all data into queue and return */
            if (q_state == Queue_ok)
                return;
        }
        else
        {
            tmp_buf_size -= i;
            p_obj->tmp_buf_remain += i;
            memset(p_obj->tmp_buf + tmp_buf_size, 0, p_obj->tmp_buf_remain);
        }
    }
    
    if (p_obj->tmp_buf_remain >= size)
    {
        /* if queue is in use or full, update temporary buff */
        memcpy(p_obj->tmp_buf + tmp_buf_size, buff, size);
        p_obj->tmp_buf_remain -= size;
    }
}

static bool SrvComTrans_DataAvailable(SrvComObj_TypeDef obj)
{
    if (!obj.init_state || (Queue.state(obj.rec_Q) == Queue_empty))
        return false;

    return true;
}

static bool SrvComTrans_Write(SrvComObj_TypeDef *obj, uint8_t *p_data, uint16_t len)
{
    if ((obj == NULL) || !obj->init_state || (p_data == NULL) || (len == 0))
        return false;

    osSemaphoreWait(obj->Tx_Irq_Sem, 0);
    if (!BspUart.send(To_BspUart_ObjPtr(obj->port_obj), p_data, len) || \
        osSemaphoreWait(obj->Tx_Irq_Sem, SRV_COM_TX_TIMEOUT))
        return false;

    return true;
}

static uint16_t SrvComTrans_GetRecData(SrvComObj_TypeDef *obj, uint8_t *p_data, uint16_t len)
{
    uint16_t queue_size = 0;
    uint16_t read_size = 0;

    if ((obj == NULL) || !obj->init_state || (p_data == NULL) || (len == 0) || (Queue.state(obj->rec_Q) == Queue_empty))
        return 0;

    queue_size = Queue.size(obj->rec_Q);
    obj->queue_inuse = true;

    if (len <= queue_size)
    {
        read_size = len;
        Queue.pop(&obj->rec_Q, p_data, len);
    }
    else
    {
        read_size = queue_size;
        Queue.pop(&obj->rec_Q, p_data, queue_size);
    }

    obj->queue_inuse = false;
    return read_size;
}
