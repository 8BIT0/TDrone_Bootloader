#ifndef __BSP_QSPI_PORT_DEF_H
#define __BSP_QSPI_PORT_DEF_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define To_QSPI_Obj(x)      (*((BspQSPI_Config_TypeDef *)x))
#define To_QSPI_ObjPtr(x)   ((BspQSPI_Config_TypeDef *)x)
#define To_QSPI_BusAPI(x)   ((BspQSpi_TypeDef *)x)

typedef struct
{
    bool init_state;
    void *p_qspi;
} BspQSPI_Config_TypeDef;

typedef struct
{
    bool (*init)(BspQSPI_Config_TypeDef *obj, void *qspi_hdl);
    bool (*tx)(BspQSPI_Config_TypeDef *obj, uint32_t addr, uint32_t cmd, uint8_t *p_data, uint32_t len);
    bool (*rx)(BspQSPI_Config_TypeDef *obj, uint32_t addr, uint32_t cmd, uint8_t *p_data, uint32_t len);
    bool (*cmd)(BspQSPI_Config_TypeDef *obj, uint32_t mode, uint32_t cyc, uint32_t nb_data, uint32_t cmd);
    bool (*polling)(BspQSPI_Config_TypeDef *obj, uint32_t mode, uint32_t cmd, uint32_t cyc, uint32_t nb_data, uint32_t match, uint32_t mask);
    bool (*memmap)(BspQSPI_Config_TypeDef *obj, uint32_t cmd);    /* for fast read */
} BspQSpi_TypeDef;

#endif