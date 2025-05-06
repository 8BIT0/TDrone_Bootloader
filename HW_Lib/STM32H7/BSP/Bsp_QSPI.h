#ifndef __BSP_QSPI_H
#define __BSP_QSPI_H

#include "Bsp_QSPI_Port_Def.h"

#define To_QSPI_Handle_Ptr(x) ((QSPI_HandleTypeDef *)x)
#define To_QSPI_Handle_Size sizeof(QSPI_HandleTypeDef)

extern BspQSpi_TypeDef BspQspi;

#endif
