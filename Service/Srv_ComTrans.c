#include "Srv_ComTrans.h"
#include "Srv_OsCommon.h"
#include "HW_Def.h"

#define SRV_COM_PORT            USART1
#define SRV_COM_BAUDRATE        460800
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
static __attribute__((section(".Perph_Section"))) uint8_t SrvCom_Tx_Buff[SRV_COM_TX_BUFF_LEN] = {0};
static __attribute__((section(".Perph_Section"))) uint8_t SrvCom_Rx_Buff[SRV_COM_RX_BUFF_LEN] = {0};
static uint8_t SrvCom_Rx_tmpBuf_1[SRV_COM_RX_BUFF_LEN];
static uint8_t SrvCom_Rx_tmpBuf_2[SRV_COM_RX_BUFF_LEN];

/* internal function */
static void SrvComTrans_Rx_Callback(uint32_t cust_data_addr, uint8_t *buff, uint16_t size);
static void SrvComTrans_Tx_Callback(uint32_t cust_data_addr, uint8_t *buff, uint16_t size);

/* external function */
static bool SrvComTrans_Init(SrvComObj_TypeDef *obj);
static bool SrvComTrans_DeInit(SrvComObj_TypeDef *obj);
static uint16_t SrvComTrans_DataAvailable(SrvComObj_TypeDef *obj);
static bool SrvComTrans_Write(SrvComObj_TypeDef *obj, uint8_t *p_data, uint16_t len);
static uint16_t SrvComTrans_GetRecData(SrvComObj_TypeDef *obj, uint8_t *p_data, uint16_t len);

/* external variable */
SrvComTrans_TypeDef SrvCom = {
    .init           = SrvComTrans_Init,
    .de_init        = SrvComTrans_DeInit,
    .write          = SrvComTrans_Write,
    .read           = SrvComTrans_GetRecData,
    .available      = SrvComTrans_DataAvailable,
};

static bool SrvComTrans_Init(SrvComObj_TypeDef *obj)
{
    bool state = true;

    if (obj == NULL)
        return false;

    obj->init_state = false;

    memset(SrvCom_Tx_Buff,     0, SRV_COM_TX_BUFF_LEN);
    memset(SrvCom_Rx_Buff,     0, SRV_COM_RX_BUFF_LEN);
    memset(SrvCom_Rx_tmpBuf_1, 0, SRV_COM_RX_BUFF_LEN);
    memset(SrvCom_Rx_tmpBuf_2, 0, SRV_COM_RX_BUFF_LEN);

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
    To_BspUart_ObjPtr(obj->port_obj)->tx_buf            = SrvCom_Tx_Buff;
    To_BspUart_ObjPtr(obj->port_obj)->tx_buf_size       = SRV_COM_TX_BUFF_LEN;
    To_BspUart_ObjPtr(obj->port_obj)->cust_data_addr    = (uint32_t)obj;

    /* init port object */
    /* create tx semaphore */
    state &= BspUart.init(To_BspUart_ObjPtr(obj->port_obj));

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

    obj->p_buf_1 = SrvCom_Rx_tmpBuf_1;
    obj->p_buf_2 = SrvCom_Rx_tmpBuf_2;
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
    uint16_t i = 0;

    if ((p_obj == NULL) || !p_obj->init_state || (buff == NULL) || (size == 0))
        return;

    p_obj->rec_cnt ++;
    p_obj->rec_size += size;

    if (!p_obj->rx_reading)
    {
        memcpy(p_obj->p_buf_1 + p_obj->buf_1_size, buff, size);
        p_obj->buf_1_size += size;
    }
    else
    {
        /* if queue is in use or full, update temporary buff */
        memcpy(p_obj->p_buf_2 + p_obj->buf_2_size, buff, size);
        p_obj->buf_2_size += size;
    }
}

static uint16_t SrvComTrans_DataAvailable(SrvComObj_TypeDef *obj)
{
    uint16_t size = 0;

    obj->rx_reading = true;
    size = obj->buf_1_size;
    obj->rx_reading = false;

    if (obj->buf_2_size)
        size += obj->buf_2_size;

    return size;
}

static bool SrvComTrans_Write(SrvComObj_TypeDef *obj, uint8_t *p_data, uint16_t len)
{
    bool state = true;

    if ((obj == NULL) || !obj->init_state || (p_data == NULL) || (len == 0))
        return false;

    osSemaphoreWait(obj->Tx_Irq_Sem, 0);
    state &= BspUart.send(To_BspUart_ObjPtr(obj->port_obj), p_data, len);
    state &= (osSemaphoreWait(obj->Tx_Irq_Sem, SRV_COM_TX_TIMEOUT) == osOK) ? true : false;

    return state;
}

static uint16_t SrvComTrans_GetRecData(SrvComObj_TypeDef *obj, uint8_t *p_data, uint16_t len)
{
    uint16_t queue_size = 0;
    uint16_t read_size = 0;
    uint16_t rec_size = len;

    if ((obj == NULL) || !obj->init_state || (p_data == NULL) || (len == 0))
        return 0;

    if (len >= obj->buf_1_size)
        rec_size = obj->buf_1_size;

    obj->rx_reading = true;
    read_size = rec_size;
    memcpy(p_data, obj->p_buf_1, rec_size);
    memset(obj->p_buf_1, 0, obj->buf_1_size);
    obj->buf_1_size = 0;
    obj->rx_reading = false;

    len -= rec_size;
    if (len && obj->buf_2_size)
    {
        rec_size = len;
        if (rec_size >= obj->buf_2_size)
            rec_size = obj->buf_2_size;
        
        read_size += rec_size;
        memcpy(p_data, obj->p_buf_2, rec_size);
        memset(obj->p_buf_2, 0, obj->buf_2_size);
        obj->buf_2_size = 0; 
    }

    return read_size;
}
